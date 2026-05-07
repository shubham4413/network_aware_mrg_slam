// AdaptiveCloudTransmitter: downsample outgoing keyframe clouds based on
// LinkQuality (/link_quality). Local clouds are not modified; only the
// copy going into GraphRos is filtered.
//
// policy = "per_cloud": each cloud is budgeted against 1/target_send_hz.
// Tries GOOD -> FAIR -> POOR and uses the first that fits. POOR is the fallback.
//
// policy = "batch": picks one tier for the whole graph exchange using
// fixed ratio estimates (good ~0.21, fair ~0.12 of original size) against
// exchange_interval.

#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mrg_slam_msgs/msg/link_quality.hpp>
#include <mrg_slam_msgs/msg/adaptive_tx_stats.hpp>

namespace mrg_slam {

class AdaptiveCloudTransmitter {
public:
    enum class Tier { EXCELLENT, GOOD, FAIR, POOR };
    enum class Policy { PER_CLOUD, BATCH };

    explicit AdaptiveCloudTransmitter( rclcpp::Node* node, const std::string& robot_name = "" ) :
        current_bandwidth_mbit_( 1000.0f ), robot_name_( robot_name ),
        logger_( node->get_logger() ), clock_( node->get_clock() )
    {
        node->declare_parameter<bool>( "adaptive_tx.enabled", true );
        node->declare_parameter<std::string>( "adaptive_tx.link_quality_topic", "/link_quality" );
        node->declare_parameter<std::string>( "adaptive_tx.policy", "per_cloud" );
        node->declare_parameter<double>( "adaptive_tx.target_send_hz", 2.0 );
        node->declare_parameter<double>( "adaptive_tx.exchange_interval", 2.0 );
        node->declare_parameter<double>( "adaptive_tx.good_leaf_size", 0.13 );
        node->declare_parameter<double>( "adaptive_tx.fair_leaf_size", 0.20 );
        node->declare_parameter<double>( "adaptive_tx.poor_leaf_size", 1.2 );
        node->declare_parameter<double>( "adaptive_tx.viz_offset_x", 0.0 );

        enabled_         = node->get_parameter( "adaptive_tx.enabled" ).as_bool();
        target_send_hz_  = static_cast<float>( node->get_parameter( "adaptive_tx.target_send_hz" ).as_double() );
        exchange_interval_ = static_cast<float>( node->get_parameter( "adaptive_tx.exchange_interval" ).as_double() );
        good_leaf_size_  = static_cast<float>( node->get_parameter( "adaptive_tx.good_leaf_size" ).as_double() );
        fair_leaf_size_  = static_cast<float>( node->get_parameter( "adaptive_tx.fair_leaf_size" ).as_double() );
        poor_leaf_size_  = static_cast<float>( node->get_parameter( "adaptive_tx.poor_leaf_size" ).as_double() );
        viz_offset_x_   = static_cast<float>( node->get_parameter( "adaptive_tx.viz_offset_x" ).as_double() );

        std::string policy_str = node->get_parameter( "adaptive_tx.policy" ).as_string();
        if( policy_str == "batch" ) {
            policy_ = Policy::BATCH;
        } else {
            policy_ = Policy::PER_CLOUD;
        }

        if( !enabled_ ) {
            RCLCPP_INFO( logger_, "AdaptiveCloudTransmitter disabled, clouds pass through untouched (baseline mode)" );
            return;
        }

        std::string lq_topic = node->get_parameter( "adaptive_tx.link_quality_topic" ).as_string();

        lq_sub_ = node->create_subscription<mrg_slam_msgs::msg::LinkQuality>(
            lq_topic, rclcpp::QoS( 10 ),
            [this]( const mrg_slam_msgs::msg::LinkQuality::SharedPtr msg ) { link_quality_callback( msg ); } );

        original_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>( "adaptive_tx/original_cloud", rclcpp::QoS( 5 ) );
        downsampled_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>( "adaptive_tx/downsampled_cloud", rclcpp::QoS( 5 ) );
        stats_pub_ = node->create_publisher<mrg_slam_msgs::msg::AdaptiveTxStats>( "adaptive_tx/stats", rclcpp::QoS( 5 ) );

        // republish last clouds at 1 Hz so rviz always has something to show
        republish_timer_ = node->create_wall_timer( std::chrono::seconds( 1 ), [this]() { republish_last_clouds(); } );

        const char* policy_name = ( policy_ == Policy::BATCH ) ? "batch" : "per_cloud";
        RCLCPP_INFO_STREAM( logger_, "AdaptiveCloudTransmitter initialized. Policy: " << policy_name
                                     << " | robot: " << robot_name_
                                     << " | topic: " << lq_topic
                                     << " | target_send_hz: " << target_send_hz_
                                     << " | exchange_interval: " << exchange_interval_
                                     << " | leaf sizes: " << good_leaf_size_ << "/" << fair_leaf_size_ << "/" << poor_leaf_size_ );
    }

    // per_cloud entry point
    sensor_msgs::msg::PointCloud2 downsample_for_transmission( const sensor_msgs::msg::PointCloud2& cloud_in )
    {
        if( !enabled_ ) {
            return cloud_in;
        }

        const float bw_mbit = current_bandwidth_mbit_.load();
        const float lat_ms  = current_latency_ms_.load();
        const float time_budget_s = 1.0f / target_send_hz_;
        const uint32_t original_bytes = static_cast<uint32_t>( cloud_in.data.size() );

        if( transmit_time_s( original_bytes, bw_mbit, lat_ms ) <= time_budget_s ) {
            RCLCPP_INFO_STREAM( logger_, "AdaptiveCloudTransmitter: " << original_bytes << " B (unchanged)"
                                         << " | bw=" << bw_mbit << " Mbps, lat=" << lat_ms
                                         << " ms | " << tier_name( Tier::EXCELLENT ) );
            store_and_publish( cloud_in, cloud_in, Tier::EXCELLENT, original_bytes, original_bytes );
            return cloud_in;
        }

        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromROSMsg( cloud_in, pcl_cloud );
        if( pcl_cloud.empty() ) {
            return cloud_in;
        }
        const auto cloud_shared = pcl_cloud.makeShared();

        struct Candidate { Tier tier; float leaf; bool approx; };
        const Candidate candidates[] = {
            { Tier::GOOD, good_leaf_size_, false },
            { Tier::FAIR, fair_leaf_size_, false },
            { Tier::POOR, poor_leaf_size_, true  },
        };

        for( int i = 0; i < 3; ++i ) {
            sensor_msgs::msg::PointCloud2 out = filter_at_leaf( cloud_shared, candidates[i].leaf, candidates[i].approx );
            out.header = cloud_in.header;
            const uint32_t sent_bytes = static_cast<uint32_t>( out.data.size() );
            const bool is_last = ( i == 2 );
            if( is_last || transmit_time_s( sent_bytes, bw_mbit, lat_ms ) <= time_budget_s ) {
                RCLCPP_INFO_STREAM( logger_, "AdaptiveCloudTransmitter: " << original_bytes << " B -> " << sent_bytes
                                             << " B | bw=" << bw_mbit << " Mbps, lat=" << lat_ms
                                             << " ms | " << tier_name( candidates[i].tier )
                                             << " (leaf " << candidates[i].leaf << " m)" );
                store_and_publish( cloud_in, out, candidates[i].tier, original_bytes, sent_bytes );
                return out;
            }
        }

        return cloud_in;
    }

    // batch entry points
    Tier select_batch_tier( uint64_t total_batch_bytes ) const
    {
        if( !enabled_ ) {
            return Tier::EXCELLENT;
        }

        const float bw_mbit = current_bandwidth_mbit_.load();
        const float lat_ms  = current_latency_ms_.load();

        auto clamp = []( uint64_t v ) -> uint32_t {
            return static_cast<uint32_t>( std::min<uint64_t>( v, UINT32_MAX ) );
        };

        if( transmit_time_s( clamp( total_batch_bytes ), bw_mbit, lat_ms ) <= exchange_interval_ ) {
            return Tier::EXCELLENT;
        }
        if( transmit_time_s( clamp( static_cast<uint64_t>( total_batch_bytes * 0.21f ) ), bw_mbit, lat_ms ) <= exchange_interval_ ) {
            return Tier::GOOD;
        }
        if( transmit_time_s( clamp( static_cast<uint64_t>( total_batch_bytes * 0.12f ) ), bw_mbit, lat_ms ) <= exchange_interval_ ) {
            return Tier::FAIR;
        }
        return Tier::POOR;
    }

    sensor_msgs::msg::PointCloud2 apply_tier( const sensor_msgs::msg::PointCloud2& cloud_in, Tier tier )
    {
        const uint32_t original_bytes = static_cast<uint32_t>( cloud_in.data.size() );

        if( !enabled_ || tier == Tier::EXCELLENT ) {
            store_and_publish( cloud_in, cloud_in, tier, original_bytes, original_bytes );
            return cloud_in;
        }

        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromROSMsg( cloud_in, pcl_cloud );

        if( pcl_cloud.empty() ) {
            return cloud_in;
        }

        const float leaf = tier_leaf_size( tier );
        pcl::PointCloud<pcl::PointXYZI> filtered;

        if( tier == Tier::GOOD || tier == Tier::FAIR ) {
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setInputCloud( pcl_cloud.makeShared() );
            vg.setLeafSize( leaf, leaf, leaf );
            vg.filter( filtered );
        } else {
            pcl::ApproximateVoxelGrid<pcl::PointXYZI> avg;
            avg.setInputCloud( pcl_cloud.makeShared() );
            avg.setLeafSize( leaf, leaf, leaf );
            avg.filter( filtered );
        }

        sensor_msgs::msg::PointCloud2 cloud_out;
        pcl::toROSMsg( filtered, cloud_out );
        cloud_out.header = cloud_in.header;

        const uint32_t sent_bytes = static_cast<uint32_t>( cloud_out.data.size() );
        RCLCPP_INFO_STREAM( logger_, "AdaptiveCloudTransmitter: " << original_bytes << " B -> " << sent_bytes
                                     << " B | bw=" << current_bandwidth_mbit_.load() << " Mbps, lat=" << current_latency_ms_.load()
                                     << " ms | " << tier_name( tier ) << " (leaf " << leaf << " m)" );

        store_and_publish( cloud_in, cloud_out, tier, original_bytes, sent_bytes );

        return cloud_out;
    }

    Policy policy() const { return policy_; }
    bool   enabled() const { return enabled_; }
    const char* tier_name( Tier t ) const
    {
        switch( t ) {
            case Tier::EXCELLENT: return "FULL";
            case Tier::GOOD: return "GOOD (VG 0.13m)";
            case Tier::FAIR: return "FAIR (VG 0.20m)";
            case Tier::POOR: return "POOR (AVG 1.2m)";
        }
        return "UNKNOWN";
    }

private:
    sensor_msgs::msg::PointCloud2 offset_cloud( const sensor_msgs::msg::PointCloud2& cloud, float offset_x )
    {
        if( std::abs( offset_x ) < 0.001f ) return cloud;

        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromROSMsg( cloud, pcl_cloud );
        for( auto& pt : pcl_cloud ) {
            pt.x += offset_x;
        }
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg( pcl_cloud, out );
        out.header = cloud.header;
        return out;
    }

    void store_and_publish( const sensor_msgs::msg::PointCloud2& original,
                            const sensor_msgs::msg::PointCloud2& downsampled,
                            Tier tier, uint32_t orig_bytes, uint32_t sent_bytes )
    {
        auto ds_viz = offset_cloud( downsampled, viz_offset_x_ );

        {
            std::lock_guard<std::mutex> lock( last_cloud_mutex_ );
            last_original_cloud_ = original;
            last_downsampled_cloud_ = ds_viz;
            has_clouds_ = true;
        }
        original_cloud_pub_->publish( original );
        downsampled_cloud_pub_->publish( ds_viz );

        total_original_bytes_ += orig_bytes;
        total_sent_bytes_ += sent_bytes;

        mrg_slam_msgs::msg::AdaptiveTxStats stats;
        stats.robot_name = robot_name_;
        stats.original_bytes = orig_bytes;
        stats.sent_bytes = sent_bytes;
        stats.tier_name = tier_name( tier );
        stats.bandwidth_mbit = current_bandwidth_mbit_.load();
        stats.latency_ms = current_latency_ms_.load();
        stats.total_original_bytes = total_original_bytes_;
        stats.total_sent_bytes = total_sent_bytes_;
        stats_pub_->publish( stats );
    }

    void republish_last_clouds()
    {
        std::lock_guard<std::mutex> lock( last_cloud_mutex_ );
        if( !has_clouds_ ) return;
        auto now = clock_->now();
        last_original_cloud_.header.stamp = now;
        last_downsampled_cloud_.header.stamp = now;
        original_cloud_pub_->publish( last_original_cloud_ );
        downsampled_cloud_pub_->publish( last_downsampled_cloud_ );
    }

    void link_quality_callback( const mrg_slam_msgs::msg::LinkQuality::SharedPtr msg )
    {
        const float bw  = static_cast<float>( msg->bandwidth_mbit );
        const float lat = static_cast<float>( msg->latency_ms );
        if( bw < 0.0f ) return;
        current_bandwidth_mbit_.store( bw );
        current_latency_ms_.store( lat );
    }

    float transmit_time_s( uint32_t bytes, float bw_mbit, float lat_ms ) const
    {
        if( bw_mbit <= 0.01f ) return 1e9f;
        const float bw_bps = bw_mbit * 1e6f;
        const float lat_s  = lat_ms / 1000.0f;
        return ( static_cast<float>( bytes ) * 8.0f ) / bw_bps + lat_s;
    }

    // voxel filter only; caller sets the output header
    sensor_msgs::msg::PointCloud2 filter_at_leaf(
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pcl_cloud,
        float leaf, bool approx ) const
    {
        pcl::PointCloud<pcl::PointXYZI> filtered;
        if( approx ) {
            pcl::ApproximateVoxelGrid<pcl::PointXYZI> avg;
            avg.setInputCloud( pcl_cloud );
            avg.setLeafSize( leaf, leaf, leaf );
            avg.filter( filtered );
        } else {
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setInputCloud( pcl_cloud );
            vg.setLeafSize( leaf, leaf, leaf );
            vg.filter( filtered );
        }
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg( filtered, out );
        return out;
    }

    float tier_leaf_size( Tier t ) const
    {
        switch( t ) {
            case Tier::EXCELLENT: return 0.0f;
            case Tier::GOOD: return good_leaf_size_;
            case Tier::FAIR: return fair_leaf_size_;
            case Tier::POOR: return poor_leaf_size_;
        }
        return 0.0f;
    }

    bool    enabled_{true};
    Policy  policy_{Policy::PER_CLOUD};
    std::string robot_name_;

    std::atomic<float> current_bandwidth_mbit_;
    std::atomic<float> current_latency_ms_{0.0f};

    float target_send_hz_;
    float exchange_interval_;
    float good_leaf_size_;
    float fair_leaf_size_;
    float poor_leaf_size_;
    float viz_offset_x_{0.0f};

    uint64_t total_original_bytes_{0};
    uint64_t total_sent_bytes_{0};

    rclcpp::Logger                                                              logger_;
    rclcpp::Clock::SharedPtr                                                    clock_;
    rclcpp::Subscription<mrg_slam_msgs::msg::LinkQuality>::SharedPtr           lq_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr                 original_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr                 downsampled_cloud_pub_;
    rclcpp::Publisher<mrg_slam_msgs::msg::AdaptiveTxStats>::SharedPtr          stats_pub_;
    rclcpp::TimerBase::SharedPtr                                                republish_timer_;

    std::mutex                    last_cloud_mutex_;
    sensor_msgs::msg::PointCloud2 last_original_cloud_;
    sensor_msgs::msg::PointCloud2 last_downsampled_cloud_;
    bool                          has_clouds_{false};
};

}  // namespace mrg_slam
