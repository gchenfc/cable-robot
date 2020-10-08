#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/symbolic/SymbolicFactor.h>
#include <cmath>
#include <boost/shared_ptr.hpp>
#include "CableFactor.h"
#include "JointLimitFactor.h"
#include "LinearDynamicsFactor.h"
#include <fstream>
namespace cable_robot{
	class LQRFactorGraph{
		private:
			std::vector<gtsam::Key> tension, point, forces, forcesAnchor, pointEE, velEE, forceEE;
			std::vector<std::vector<gtsam::Key>> intermediateForces;
			std::vector<std::vector<double>> EEpath;
				
			gtsam::Point3 anchorPoints[4] = {
				gtsam::Point3(0,0,0),
				gtsam::Point3(1,0,0),
				gtsam::Point3(0,1,0),
				gtsam::Point3(1,1,0)
			};
			gtsam::Pose3 EEinitLoc;
			int mass;
			std::vector<double> tension_lim;
			gtsam::NonlinearFactorGraph graph;
			gtsam::Values linPoint;

			std::vector<double> line_space(int start,int end,  int n) {
				std::vector<double> out;
				if (n == 0) {
					return out;
				}else if(n == 1){
					out.push_back(end);
					return out;
				}else if(n == 2) {
					out.push_back(start);
					out.push_back(end);
					return out;
				}
				out.push_back(start);
				double buff = start;
				double delta = ((double)(end-start))/(n-2);
				for (int i = 0; i < (n-2); i++) {
					buff += delta;
					out.push_back(buff);
				}
				out.push_back(end);
				return out;
			};
			std::vector<std::vector<double>> path_builder(std::vector<double> times) {
				std::vector<std::vector<double>> out;
				for (int i = 0; i < times.size(); i++) {
					static const double arr[3] = {cos(i/3*2*M_PI)*0.25+0.5,sin(i/3*2*M_PI)*0.25+0.5,0};
					std::vector<double> vec(arr, arr + sizeof(arr) / sizeof(arr[0]));
					out.push_back(vec);
				};
				return out;
			};
			void add_level_to_intermediateForces (int len, std::vector<std::vector<gtsam::Key>>& target) {
				for(int i = 0; i < len; i++) {
					std::vector<gtsam::Key> level(0,2);
					target.push_back(level);
				}
			};
			gtsam::Vector vectorBuilder(int n, int l) {
				gtsam::Vector out(l);
				for ( int i = 0; i < l; i++){
					out(i) = n;
				}
				return out;

			}
		public:
			void run() {
				std::cout << "start" << std::endl;
				//std::vector<std::shared_ptr<gtsam::Symbol>> forces;
				//graph = gtsam::NonlinearFactorGraph();
				mass = 1;
				std::vector<double> timeline = line_space(0,3,50);
				tension_lim.push_back(0.1);
				tension_lim.push_back(10);
				EEpath =  path_builder(timeline);
				//add_level_to_intermediateForces(timeline.size(),intermediateForces);
				std::cout << "timeline ";
				std::cout << timeline.size() << std::endl; 
				for (int ti = 0; ti < timeline.size(); ti ++ ){
					for (int ci = 0; ci < 4; ci ++) {
						int ind  = ti*4 + ci;
						tension.push_back(gtsam::symbol('t', ind));
				        point.push_back(gtsam::symbol('p', ind));
				        forces.push_back(gtsam::symbol('f', ind));
				        forcesAnchor.push_back(gtsam::symbol('g', ind));
					}
					pointEE.push_back(gtsam::symbol('e', ti));
				    velEE.push_back(gtsam::symbol('v', ti));
				    forceEE.push_back(gtsam::symbol('F', ti));
				    intermediateForces.push_back(std::vector<gtsam::Key>());
				    intermediateForces[ti].push_back(gtsam::symbol('i', (ti)*2 + 0));
				    intermediateForces[ti].push_back(gtsam::symbol('i', (ti)*2 + 1));
				}
				std::cout << "finish building forces" << std::endl;
				for (int ti = 0; ti < timeline.size(); ti ++ ){
					for (int ci = 0; ci < 4; ci ++) {
						int ind  = ti*4 + ci;

			            graph.push_back(boost::make_shared<manipulator::CableFactor>(tension[ind],point[ind]
			            	, pointEE[ti], forcesAnchor[ind], forces[ind], gtsam::noiseModel::Isotropic::Variance(6, 0.1)));
			            graph.push_back(boost::make_shared<manipulator::JointLimitFactor>(tension[ind], gtsam::noiseModel::Isotropic::Variance(1, 0.1)
			            	,tension_lim[0], tension_lim[1],0));
			            graph.push_back(boost::make_shared<gtsam::PriorFactor<gtsam::Point3>>(point[ind], anchorPoints[ci],gtsam::noiseModel::Isotropic::Variance(3, 0.1)));
			            graph.push_back(boost::make_shared<manipulator::JointLimitFactor>(tension[ind], gtsam::noiseModel::Isotropic::Variance(1, 1000),0,0,0));
					}
					gtsam::Matrix eyes[9] = {gtsam::Matrix(3,3),gtsam::Matrix(3,3),gtsam::Matrix(3,3),gtsam::Matrix(3,3),gtsam::Matrix(3,3),gtsam::Matrix(3,3),gtsam::Matrix(3,3),gtsam::Matrix(3,3),gtsam::Matrix(3,3)};
					//gtsam::Vector b[3] = {{0,0,0},{0,0,0},{0,0,0}}
					graph.push_back(boost::make_shared<gtsam::PriorFactor<gtsam::Point3>>(pointEE[ti],gtsam::Point3(EEpath[ti][0],EEpath[ti][1],EEpath[ti][2]), gtsam::noiseModel::Isotropic::Variance(3, 1.0)));
					
					
					for(int tmp = 0; tmp < 4; tmp++){
						 linPoint.insert(forces[ti*4 + tmp], vectorBuilder(0,3));
					}

					linPoint.insert(intermediateForces[ti][0], vectorBuilder(0,3));
				    linPoint.insert(intermediateForces[ti][1], vectorBuilder(0,3));
				    linPoint.insert(forceEE[ti], vectorBuilder(0,3));
				    
				    graph.push_back(boost::make_shared<gtsam::LinearContainerFactor>(gtsam::JacobianFactor(
				        forces[ti*4 + 0], eyes[0],
				        forces[ti*4 + 1], eyes[1],
				        intermediateForces[ti][0], -eyes[2],
				        vectorBuilder(0,3), gtsam::noiseModel::Isotropic::Variance(3, 0.1)), linPoint));
				    graph.push_back(boost::make_shared<gtsam::LinearContainerFactor>(gtsam::JacobianFactor(
				        forces[ti*4 + 0], eyes[3],
				        forces[ti*4 + 1], eyes[4],
				        intermediateForces[ti][1], -eyes[5],
				        vectorBuilder(0,3), gtsam::noiseModel::Isotropic::Variance(3, 0.1)), linPoint));
				    graph.push_back(boost::make_shared<gtsam::LinearContainerFactor>(gtsam::JacobianFactor(
				        intermediateForces[ti][0], eyes[6],
				        intermediateForces[ti][1], eyes[7],
				        forceEE[ti], -eyes[8],
				        vectorBuilder(0,3), gtsam::noiseModel::Isotropic::Variance(3, 0.1)), linPoint));
				    
				    if (ti<(timeline.size()-1)){
				    	graph.push_back(boost::make_shared<manipulator::LinearDynamicsFactor>( 
					        pointEE[ti], velEE[ti], forceEE[ti]/mass, 
					        pointEE[ti+1], velEE[ti+1], 
					        gtsam::noiseModel::Isotropic::Variance(6, 0.1), timeline[1]-timeline[0]));
				    }

				}

				std::cout << "finsih graph" << std::endl;
				gtsam::Values initialEstimate;
				for (int ti  = 0; ti < timeline.size(); ti++) {
				    for (int ci = 0; ci < 4;ci++){
				        int ind = ti*4 + ci;
				        initialEstimate.insertDouble(tension[ind], 1); 
				        initialEstimate.insert(point[ind], anchorPoints[ci]);
				        initialEstimate.insert(forces[ind], vectorBuilder(0,3));
				        initialEstimate.insert(forcesAnchor[ind], vectorBuilder(0,3));
				    }
				    initialEstimate.insert(pointEE[ti], gtsam::Point3(EEpath[ti][0],EEpath[ti][1],EEpath[ti][2]));
				    initialEstimate.insert(velEE[ti], vectorBuilder(0,3));
				    initialEstimate.insert(forceEE[ti], vectorBuilder(0,3));
				    initialEstimate.insert(intermediateForces[ti][0], vectorBuilder(0,3));
				    initialEstimate.insert(intermediateForces[ti][1], vectorBuilder(0,3));
				}
				std::cout << "optimizing...\n" << std::endl;
				gtsam::LevenbergMarquardtParams params;
				
				params.setlambdaInitial(1000);
				
				gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
				gtsam::Values result = optimizer.optimizeSafely();
				
				std::cout << "done optimizing\n" << std::endl;

				std::ofstream dest;
				dest.open("vectorout.txt");
				for (int ci = 0; ci < 4; ci++) {
					dest << "anchorpoint#" << ci << std::endl;
					dest << anchorPoints[ci] << std::endl;
				}
				for (int ti = 0; ti < timeline.size(); ti++) {
					dest << "timeline#" << ti << std::endl;
					dest << result.at<gtsam::Point3>(pointEE[ti]).vector() << std::endl;
					for (double n : EEpath[ti]){
						dest << n << "_";
					}
					dest << std::endl;
				}
				dest << "++++++++++++++++++++++++++" <<std::endl;
			    for(int ti = 0; ti < timeline.size(); ti++){

			    	for (int ci = 0; ci < 4; ci++) {
			    		int ind = ti*4 + ci;
			    		dest << "timeline#" << ti << std::endl;
			    		dest << "anchorpoint#" << ci << std::endl;
			    		dest << result.at<gtsam::Point3>(point[ind]).vector() << std::endl;
			    		dest << result.at<double>(tension[ind]) << std::endl;
			    		dest << result.at<gtsam::Vector>(forces[ind]) << std::endl;
			    		/*
				        anchorVertsExp[ci] = anchorPoints[ci].vector();
				        anchorVertsAct(ci, :) = result.atPoint3(point(i)).vector();
				        EEVertsExp(ci, :) = EEpath(ti, :);
				        EEVertsAct(ci, :) = result.atPoint3(pointEE(ti)).vector();
				        EEwrenches(ci, :) = [0;0;0;result.atVector(forces(i))]';
				        tensions(ci) = result.atDouble(tension(i));	
				        */ 
				        
			    	}
			    }
			    dest.close();
			};
	};

}