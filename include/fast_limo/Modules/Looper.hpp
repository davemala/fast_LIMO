/*
 Copyright (c) 2024 Oriol Martínez @fetty31

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __FASTLIMO_LOOPER_HPP__
#define __FASTLIMO_LOOPER_HPP__

#include "fast_limo/Common.hpp"
#include <pcl/point_types.h>
#include "fast_limo/Objects/State.hpp"
#include "fast_limo/Utils/LoopConfig.hpp"
#include "fast_limo/Utils/GNSStf.hpp"

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "Scancontext/scancontext.hpp"

using namespace fast_limo;

class fast_limo::Looper {

    // VARIABLES

    private:
            // PoseGraph
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values init_estimates;
        gtsam::ISAM2* iSAM_;
        gtsam::Pose3 out_estimate;

        bool priorAdded;
        uint64_t global_idx;

        std::mutex graph_mtx;

        gtsam::noiseModel::Diagonal::shared_ptr prior_noise;
        gtsam::noiseModel::Diagonal::shared_ptr odom_noise;
        gtsam::noiseModel::Base::shared_ptr gnss_noise;
        gtsam::noiseModel::Base::shared_ptr loop_noise;

            // Config struct
        LoopConfig config;

            // Loop Closure
        std::unique_ptr<ScanContext> sc_ptr_;
        pcl::IterativeClosestPoint<PointType, PointType> icp;

            // Keyframes
        boost::circular_buffer<std::pair<State, pcl::PointCloud<PointType>::Ptr>> keyframes;
        std::mutex kf_mtx;

            // GNSS transformer
        GNSStf gnss_tf;

            // Aux var.
        bool initFlag;
        bool ICPcorrect;
        State last_kf;
        Eigen::Vector3d last_enu;
        fast_limo::State prior_state;

        pcl::PointCloud<PointType>::Ptr icp_source_pc;
        pcl::PointCloud<PointType>::Ptr icp_target_pc;
        pcl::PointCloud<PointType>::Ptr icp_result_pc;

    // FUNCTIONS

    public:
        Looper();
        ~Looper();

        void init(LoopConfig& cfg); // To DO: add config struct as input

        State get_state();
        State get_last_odom();
        void get_state(State& s);

        std::vector<double> getPoseCovariance(); // get iSAM covariances
        std::vector<double> getTwistCovariance();// get iSAM covariances

        std::vector< std::pair<State, 
                    pcl::PointCloud<PointType>::Ptr> > getKeyFrames();
        
        float getScanContextResult();
        int getScanContextIndex();

        pcl::PointCloud<PointType>::Ptr getICPsource();
        pcl::PointCloud<PointType>::Ptr getICPtarget();
        pcl::PointCloud<PointType>::Ptr getICPresult();
        bool hasICPconverged();

        bool solve();
        void check_loop();

        void update(State s, pcl::PointCloud<PointType>::Ptr&);
        void update(State s);
        void update(double lat, double lng, double alt);

    private:
        void updateGraph(gtsam::Pose3 pose);
        void updateLoopClosure(gtsam::Pose3 pose, int id0, int idN);

        gtsam::Pose3 fromLIMOtoGTSAM(const State& s);

        bool time2update(const State& s);
        bool time2update(Eigen::Vector3d& enu);
        bool time2loop();

        void updateKeyFrames(gtsam::Values* graph_estimate);

        gtsam::Pose3 run_icp(int id0, int idN);

    // SINGLETON 

    public:
        static Looper& getInstance(){
            static Looper* loop = new Looper();
            return *loop;
        }

    private:
        // Disable copy/move functionality
        Looper(const Looper&) = delete;
        Looper& operator=(const Looper&) = delete;
        Looper(Looper&&) = delete;
        Looper& operator=(Looper&&) = delete;

};

#endif