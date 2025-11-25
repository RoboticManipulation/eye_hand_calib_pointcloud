#include <iostream>

#pragma   push_macro("min")  
#pragma   push_macro("max")  
#undef   min  
#undef   max  

#include <pcl/common/transforms.h>  
#include <pcl/io/pcd_io.h> 
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/random_sample.h>

#include <limbo/bayes_opt/boptimizer.hpp>
#include <limbo/acqui/ei.hpp>
#include <limbo/serialize/text_archive.hpp>

#include <limbo/kernel/expSE3.hpp>
#include <limbo/opt/nlopt_grad.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/types.hpp"
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

#include <algorithm>

#include "GNsolver.h"

#include <filesystem>
#include <limbo/opt/nlopt_no_grad.hpp>
#include <thread>
#include <chrono>
#include <fstream>
#include <numeric>
#include <regex>
#include <vector>

// Global vectors to store multi-view point clouds
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudsOriginal;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudsSubsampled;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudsSubsampledTransformed;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudsTransformed;
std::vector<Sophus::SE3d> robotPoses;
std::vector<Eigen::Matrix3d> robotRotations;
std::vector<Eigen::Vector3d> robotTranslations;

//search space of Bayesian optimization, eye in hand
double sideLength = 0.2;
Eigen::Vector3d offset(0, 0, 0);

//search space of Bayesian optimization, eye to hand
/*double sideLength = 0.4;
Eigen::Vector3d offset(-0.6, -0.2, 0.2)*/;

//trim ratio and convergence threshold
double trimRatio = 0.9;
double convThresh = 0.0001;

//Flag for eye in hand or eye to hand. True for eye in hand, false for eye to hand
bool eyeinhand = true;

using namespace limbo;
using namespace std;

//Parameters setting for Bayesian Optimization
struct Params {
    //kernel hyper parameters update frequency in itertions
	struct bayes_opt_boptimizer : public defaults::bayes_opt_boptimizer {
	  BO_PARAM(int, hp_period, 10)         
	};

	// depending on which internal optimizer we use, we need to import different parameters
#ifdef USE_NLOPT
	struct opt_nloptnograd : public defaults::opt_nloptnograd {
	};
	struct opt_nloptgrad : public defaults::opt_nloptgrad {
	};

#elif defined(USE_LIBCMAES)
	struct opt_cmaes : public defaults::opt_cmaes {
	};
#else
	struct opt_gridsearch : public defaults::opt_gridsearch {
		BO_PARAM(int, bins, 2)
	};
#endif

	// enable / disable the writing of the result files
	struct bayes_opt_bobase : public defaults::bayes_opt_bobase {
		BO_PARAM(int, stats_enabled, false);
	};

	struct kernel : public defaults::kernel {
		BO_PARAM(double, noise, 0);
	};

	// we use 50 random samples to initialize the algorithm
	struct init_randomsampling {
		BO_PARAM(int, samples, 50);
	};

	// we stop after 50 iterations
	struct stop_maxiterations {
		BO_PARAM(int, iterations, 50);
	};

	struct acqui_ei : public defaults::acqui_ei {
	};

	struct opt_rprop : public defaults::opt_rprop {
	};

	struct kernel_expSE3 : public defaults::kernel_expSE3 {
	};
};

//Optimization objective of Bayesian Optimization, which is the mean of squared distance between corresponding points, i.e. MSE or E(u) in the paper.
struct Eval {
	// number of input dimension (x.size())
	BO_PARAM(size_t, dim_in, 6);
	// number of dimensions of the result 
	BO_PARAM(size_t, dim_out, 1);

	// the function to be optimized
	Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
	{
		//Set search space. All dimensions of x are in [0,1] by default, so rescale the hyper rectangle.
		Sophus::Vector3d Rot = (x.head(3) - Eigen::Matrix<double, 3, 1>::Constant(0.5)) * 2 * 3.1415926;  //rotation
		Eigen::Vector3d t = (x.tail(3) - Eigen::Matrix<double, 3, 1>::Constant(0.5)) * sideLength + offset; // translation

		//Convert the sample point into SE(3).
		Eigen::Matrix4d HE = Eigen::Matrix4d::Identity();
		HE.block(0, 0, 3, 3) = Sophus::SO3d::exp(Rot).matrix();
		HE.block(0, 3, 3, 1) = t;

		//Transform point clouds with HE and corresponding robot poses.
		for (size_t i = 0; i < cloudsSubsampled.size(); ++i) {
			pcl::transformPointCloud(*cloudsSubsampled[i], *cloudsSubsampledTransformed[i], 
									 (robotPoses[i].matrix() * HE).cast<float>());
		}

		std::vector<float> pointDistance(1);
		std::vector<int> pointIdx(1);
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		std::vector<float> distances;

		//Pair point clouds captured before and after each robot motion. 
		//In each pair, for each point in the smaller point cloud, we find its corresponding closest point in the other point cloud.
		for (size_t pair = 0; pair < cloudsSubsampled.size() - 1; ++pair) {
			auto& cloud1 = cloudsSubsampledTransformed[pair];
			auto& cloud2 = cloudsSubsampledTransformed[pair + 1];
			
			if (cloudsSubsampled[pair]->size() < cloudsSubsampled[pair + 1]->size()) {
				kdtree.setInputCloud(cloud2);
				for (size_t i = 0; i < cloud1->points.size(); ++i) {
					kdtree.nearestKSearch(cloud1->points[i], 1, pointIdx, pointDistance);
					distances.push_back(pointDistance[0]);
				}
			} else {
				kdtree.setInputCloud(cloud1);
				for (size_t i = 0; i < cloud2->points.size(); ++i) {
					kdtree.nearestKSearch(cloud2->points[i], 1, pointIdx, pointDistance);
					distances.push_back(pointDistance[0]);
				}
			}
		}

		std::sort(distances.begin(), distances.end());
		int numCorres = ceil(distances.size() * trimRatio);

		// Calculate MSE. Limbo always maximizes, so we take its opposite.
		double y = -1 * accumulate(distances.begin(), distances.begin() + numCorres, 0.0) / numCorres;

		// we return a 1-dimensional vector
		return tools::make_vector(y);
	}
};

// Natural sort comparator for filenames with numbers
bool naturalSort(const std::string& a, const std::string& b) {
	std::filesystem::path pathA(a);
	std::filesystem::path pathB(b);
	std::string nameA = pathA.filename().string();
	std::string nameB = pathB.filename().string();
	
	// Extract numbers from filenames
	std::regex numRegex(R"(\d+)");
	std::smatch matchA, matchB;
	
	if (std::regex_search(nameA, matchA, numRegex) && 
		std::regex_search(nameB, matchB, numRegex)) {
		int numA = std::stoi(matchA.str());
		int numB = std::stoi(matchB.str());
		return numA < numB;
	}
	
	// Fallback to lexicographic sort if no numbers found
	return nameA < nameB;
}

int main(int argc, char** argv) {
	// Read point clouds
	std::string path = (argc > 1) ? std::string(argv[1]) : std::string("./data/David");
	if (!std::filesystem::exists(path)) {
		std::cerr << "[error] Data folder does not exist: " << path << std::endl;
		return 1;
	}

	// Find all PCD files matching pattern (view*.pcd or view*d.pcd)
	std::vector<std::string> pcdFiles;
	std::regex pcdPattern(R"(view\d+d?\.pcd)", std::regex::icase);
	
	for (const auto& entry : std::filesystem::directory_iterator(path)) {
		if (entry.is_regular_file()) {
			std::string filename = entry.path().filename().string();
			if (std::regex_match(filename, pcdPattern)) {
				pcdFiles.push_back(entry.path().string());
			}
		}
	}

	// Sort files naturally (by numeric order)
	std::sort(pcdFiles.begin(), pcdFiles.end(), naturalSort);

	if (pcdFiles.size() < 2) {
		std::cerr << "[error] Need at least 2 point cloud files (view*.pcd). Found: " << pcdFiles.size() << std::endl;
		return 1;
	}

	std::cout << "Found " << pcdFiles.size() << " point cloud files:" << std::endl;
	for (const auto& f : pcdFiles) {
		std::cout << "  - " << std::filesystem::path(f).filename().string() << std::endl;
	}

	// Load point clouds
	for (const auto& file : pcdFiles) {
		auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		if (pcl::io::loadPCDFile(file, *cloud) < 0) {
			std::cerr << "[error] Could not read file: " << file << std::endl;
			return 1;
		}
		cloudsOriginal.push_back(cloud);
		cloudsSubsampled.push_back(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>());
		cloudsSubsampledTransformed.push_back(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>());
		cloudsTransformed.push_back(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>());
	}

	// Down sample for Bayesian Optimization and trimming distance calculation
	float sampleRatio = 0.1;
	pcl::RandomSample<pcl::PointXYZ> rs;
	for (size_t i = 0; i < cloudsOriginal.size(); ++i) {
		rs.setInputCloud(cloudsOriginal[i]);
		rs.setSample(ceil(cloudsOriginal[i]->size() * sampleRatio));
		rs.filter(*cloudsSubsampled[i]);
	}

	// Read robot poses. If eye-to-hand scenario then take the inverse
	std::fstream in;
	in.open(path + "/RobotPoses.dat", std::ios::in);
	if (!in.is_open()) {
		std::cerr << "[error] can not find " << (path + "/RobotPoses.dat") << std::endl;
		return 1;
	}

	std::vector<std::vector<double>> roboPoseData;
	std::string buff;
	while (getline(in, buff)) {
		std::vector<double> nums;
		char* s_input = (char*)buff.c_str();
		const char* split = ",";
		char* p = strtok(s_input, split);
		while (p != NULL) {
			nums.push_back(atof(p));
			p = strtok(NULL, split);
		}
		if (nums.size() == 6) {
			roboPoseData.push_back(nums);
		}
	}
	in.close();

	if (roboPoseData.size() != cloudsOriginal.size()) {
		std::cerr << "[error] Number of robot poses (" << roboPoseData.size() 
				  << ") doesn't match number of point clouds (" << cloudsOriginal.size() << ")" << std::endl;
		return 1;
	}

	// Convert robot poses to SE(3)
	for (size_t i = 0; i < roboPoseData.size(); ++i) {
		Eigen::Matrix3d R;
		R = Eigen::AngleAxisd(roboPoseData[i][2], Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(roboPoseData[i][1], Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(roboPoseData[i][0], Eigen::Vector3d::UnitX());

		Eigen::Vector3d t(roboPoseData[i][3] / 1000, roboPoseData[i][4] / 1000, roboPoseData[i][5] / 1000);

		Sophus::SE3d T(R, t);
		if (eyeinhand) {
			robotPoses.push_back(T);
			robotRotations.push_back(R);
			robotTranslations.push_back(t);
		} else {
			robotPoses.push_back(T.inverse());
			robotRotations.push_back(R.inverse());
			robotTranslations.push_back(-R.inverse() * t);
		}
	}

	cout << "Data is read into memory" << endl;
	cout << "Number of views: " << cloudsOriginal.size() << endl;

	cout << "Start BO-IA" << endl;
	//Use modified kernel ExpSE3
	using Kernel2_t = kernel::ExpSE3<Params>;

	//Use the mean of already sampled data as the mean of prior distribution
	using Mean_t = mean::Data<Params>;
	using GP_t = model::GP<Params, Kernel2_t, Mean_t, model::gp::KernelLFOpt<Params, opt::Rprop<Params>>>;

	//Use Expected Improvement as acquisition function
	using Acqui_t = acqui::EI<Params, GP_t>;
	using acqui_opt_t = opt::NLOptNoGrad<Params>;
	bayes_opt::BOptimizer<Params, modelfun<GP_t>, acquifun<Acqui_t>, acquiopt<acqui_opt_t>> boptimizer;
	boptimizer.optimize(Eval());

	//Take the best sample as the initial guess of hand-eye relation
	Eigen::Matrix<double, 6, 1> best = boptimizer.best_sample();
	Eigen::Matrix4d HE = Eigen::Matrix4d::Identity();
	Sophus::Vector3d Rotbest = (best.head(3) - Eigen::Matrix<double, 3, 1>::Constant(0.5)) * 2 * 3.1415926;
	Eigen::Vector3d tbest = (best.tail(3) - Eigen::Matrix<double, 3, 1>::Constant(0.5)) * sideLength + offset;
	HE.block(0, 0, 3, 3) = Sophus::SO3d::exp(Rotbest).matrix();
	HE.block(0, 3, 3, 1) = tbest;

	//Some AA-ICPv parameter initialization
	int CorresNum = 0;
	double SquareDistSum = 0.0;
	int counter = 0;
	double Err = 0;
	double ErrPrev = 0;
	double ratio = 0;
	double distanceThresh = 0;

	Eigen::Matrix3d HER = HE.block(0, 0, 3, 3);
	Eigen::Vector3d HEt = HE.block(0, 3, 3, 1);
	Eigen::Matrix<double, 6, 1> para;

	int m = 1;

	Eigen::Matrix<double, 6, Eigen::Dynamic> u(6, 0);
	Eigen::Matrix<double, 6, Eigen::Dynamic> g(6, 0);
	Eigen::Matrix<double, 6, Eigen::Dynamic> f(6, 0);

	Sophus::Vector6d u_k;
	Sophus::Vector6d u_next;

	Sophus::Vector6d u_0;
	u_0.head(3) = Sophus::SO3d::log(HER);
	u_0.tail(3) = HEt;

	u.conservativeResize(u.rows(), u.cols() + 1);
	u.col(0) = u_0;

	//Start AA-ICPv
	cout << "Start AA-ICPv" << endl;

	while (true) {
		counter = counter + 1;

		CorresNum = 0;
		SquareDistSum = 0;

		para << 0, 0, 0, HEt(0), HEt(1), HEt(2);

		//Calculate trimming distance threshold
		std::vector<int> pointIdx(1);
		std::vector<float> pointDistance(1);
		std::vector<float> distances;
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

		// Transform all subsampled clouds
		for (size_t i = 0; i < cloudsSubsampled.size(); ++i) {
			pcl::transformPointCloud(*cloudsSubsampled[i], *cloudsSubsampledTransformed[i], 
									 (robotPoses[i].matrix() * HE).cast<float>());
		}

		// Calculate distances between consecutive pairs
		for (size_t pair = 0; pair < cloudsSubsampled.size() - 1; ++pair) {
			auto& cloud1 = cloudsSubsampledTransformed[pair];
			auto& cloud2 = cloudsSubsampledTransformed[pair + 1];
			
			if (cloudsSubsampled[pair]->size() < cloudsSubsampled[pair + 1]->size()) {
				kdtree.setInputCloud(cloud2);
				for (size_t i = 0; i < cloud1->points.size(); ++i) {
					kdtree.nearestKSearch(cloud1->points[i], 1, pointIdx, pointDistance);
					distances.push_back(pointDistance[0]);
				}
			} else {
				kdtree.setInputCloud(cloud1);
				for (size_t i = 0; i < cloud2->points.size(); ++i) {
					kdtree.nearestKSearch(cloud2->points[i], 1, pointIdx, pointDistance);
					distances.push_back(pointDistance[0]);
				}
			}
		}

		std::sort(distances.begin(), distances.end());
		distanceThresh = distances[ceil(distances.size() * trimRatio)];

		// Transform all original clouds
		for (size_t i = 0; i < cloudsOriginal.size(); ++i) {
			pcl::transformPointCloud(*cloudsOriginal[i], *cloudsTransformed[i], 
									 (robotPoses[i].matrix() * HE).cast<float>());
		}

		std::vector<Eigen::Vector3d> p, q;
		std::vector<int> idx;

		// Find correspondences for all consecutive pairs with trimming
		for (size_t pair = 0; pair < cloudsOriginal.size() - 1; ++pair) {
			auto& cloud1Orig = cloudsOriginal[pair];
			auto& cloud2Orig = cloudsOriginal[pair + 1];
			auto& cloud1T = cloudsTransformed[pair];
			auto& cloud2T = cloudsTransformed[pair + 1];
			
			if (cloud1Orig->size() < cloud2Orig->size()) {
				kdtree.setInputCloud(cloud2T);
				for (size_t i = 0; i < cloud1T->points.size(); ++i) {
					kdtree.nearestKSearch(cloud1T->points[i], 1, pointIdx, pointDistance);
					if (pointDistance[0] < distanceThresh) {
						p.push_back(cloud1Orig->points[i].getVector3fMap().cast<double>());
						q.push_back(cloud2Orig->points[pointIdx[0]].getVector3fMap().cast<double>());
						CorresNum++;
						SquareDistSum += pointDistance[0];
					}
				}
			} else {
				kdtree.setInputCloud(cloud1T);
				for (size_t i = 0; i < cloud2T->points.size(); ++i) {
					kdtree.nearestKSearch(cloud2T->points[i], 1, pointIdx, pointDistance);
					if (pointDistance[0] < distanceThresh) {
						p.push_back(cloud1Orig->points[pointIdx[0]].getVector3fMap().cast<double>());
						q.push_back(cloud2Orig->points[i].getVector3fMap().cast<double>());
						CorresNum++;
						SquareDistSum += pointDistance[0];
					}
				}
			}
			idx.push_back(CorresNum);
		}

		//MSE of current iterate
		Err = SquareDistSum / CorresNum;

		if (counter == 1) { //Anderson Accleration initialization
			//Solve least square problem in (1)
			Sophus::Vector6d delta;
			GNcalculation(p, q, idx, robotRotations, robotTranslations, CorresNum, HER, HEt, delta);
			para = para + delta;

			HER = Sophus::SO3d::exp(para.head(3)).matrix() * HER;
			HEt = para.tail(3);
			HE.block(0, 0, 3, 3) = HER;
			HE.block(0, 3, 3, 1) = HEt;

			Sophus::Vector6d u_1;
			u_1.head(3) = Sophus::SO3d::log(HER);
			u_1.tail(3) = HEt;
			u.conservativeResize(u.rows(), u.cols() + 1);
			u.col(1) = u_1;
			g.conservativeResize(g.rows(), g.cols() + 1);
			g.col(0) = u_1;
			f.conservativeResize(f.rows(), f.cols() + 1);
			f.col(0) = u_1 - u_0;
			u_next = u_1;
			u_k = u_1;

			m = m + 1;
			ErrPrev = Err;

		} else {
			ratio = Err / ErrPrev;

			// Failure handling
			if (ratio > 1.03) {
				//Use the last trustworthy G call as new iterate then reset history
				u.col(u.cols() - 1) = g.col(g.cols() - 1);
				HER = Sophus::SO3d::exp(u.col(u.cols() - 1).head(3)).matrix();
				HEt = u.col(u.cols() - 1).tail(3);
				HE.block(0, 0, 3, 3) = HER;
				HE.block(0, 3, 3, 1) = HEt;
				u_k = u.col(u.cols() - 1);

				m = 2;
				ErrPrev = 100;

			} else {
				// Anderson Acceleration
				//Solve least square problem in (1)
				Sophus::Vector6d delta;
				GNcalculation(p, q, idx, robotRotations, robotTranslations, CorresNum, HER, HEt, delta);
				para = para + delta;

				HER = Sophus::SO3d::exp(para.head(3)).matrix() * HER;
				HEt = para.tail(3);

				Sophus::Vector6d g_k;
				g_k.head(3) = Sophus::SO3d::log(HER);
				g_k.tail(3) = HEt;
				g.conservativeResize(g.rows(), g.cols() + 1);
				g.col(g.cols() - 1) = g_k;
				Sophus::Vector6d f_k = g_k - u_k;

				ErrPrev = Err;

				//Check convergence
				if (f_k.norm() < convThresh) {
					break;
				}

				f.conservativeResize(f.rows(), f.cols() + 1);
				f.col(f.cols() - 1) = f_k;

				Eigen::Matrix<double, 6, Eigen::Dynamic> f_recent = f.rightCols(min(m, 5));
				Eigen::Matrix<double, 6, Eigen::Dynamic> A = f_recent.leftCols(f_recent.cols() - 1);
				A *= -1;
				A += f_recent.rightCols(1) * Eigen::Matrix<double, Eigen::Dynamic, 1>::Constant(f_recent.cols() - 1, 1).transpose();
				Eigen::Matrix<double, Eigen::Dynamic, 1> alphas = A.colPivHouseholderQr().solve(f_recent.rightCols(1));
				alphas.conservativeResize(alphas.size() + 1);
				alphas[alphas.size() - 1] = 0;
				alphas[alphas.size() - 1] = 1 - alphas.sum();

				u_next = g.rightCols(min(m, 5)) * alphas;
				u.conservativeResize(u.rows(), u.cols() + 1);
				u.col(u.cols() - 1) = u_next;

				u_k = u_next;

				HER = Sophus::SO3d::exp(u_next.head(3)).matrix();
				HEt = u_next.tail(3);
				HE.block(0, 0, 3, 3) = HER;
				HE.block(0, 3, 3, 1) = HEt;

				m = m + 1;
			}
		}
	}

	std::cout << "*********" << endl;
	std::cout << "*********" << endl;
	std::cout << "Hand-eye relation is" << endl;
	std::cout << HE << endl;
	std::cout << "*********" << endl;
	std::cout << "*********" << endl;

	// Save hand–eye matrix to CSV
	{
		std::ofstream ofs(path + "/handeye_matrix.csv");
		ofs.setf(std::ios::fixed); ofs.precision(9);
		for (int r = 0; r < 4; ++r) {
			ofs << HE(r,0) << "," << HE(r,1) << "," << HE(r,2) << "," << HE(r,3) << "\n";
		}
		std::cout << "Saved hand–eye CSV: " << (path + "/handeye_matrix.csv") << std::endl;
	}

	// Final registration - transform all clouds
	for (size_t i = 0; i < cloudsOriginal.size(); ++i) {
		pcl::transformPointCloud(*cloudsOriginal[i], *cloudsTransformed[i], 
								 (robotPoses[i].matrix() * HE).cast<float>());
	}

	// Build a merged cloud for saving/inspection
	auto CloudAll = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	for (const auto& cloud : cloudsTransformed) {
		if (cloud && !cloud->empty()) {
			*CloudAll += *cloud;
		}
	}

	// Save the merged registered point cloud
	if (!CloudAll->empty()) {
		pcl::io::savePCDFileBinary(path + "/registered_merged.pcd", *CloudAll);
		std::cout << "Saved merged cloud: " << (path + "/registered_merged.pcd") << std::endl;
	}

	// --- Off-screen rendering for screenshot only ---
	// std::cout << "Generating visualization screenshot (no window)...\n";

	// auto viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>("RegHEC result", false);

	// viewer->setBackgroundColor(0, 0, 0);
	// viewer->addCoordinateSystem(0.05);

	// // Add transformed views in distinct colors
	// std::vector<std::tuple<int, int, int>> colors = {
	// 	{255, 60, 60}, {60, 255, 60}, {60, 60, 255}, {255, 255, 60},
	// 	{255, 60, 255}, {60, 255, 255}, {255, 140, 60}, {140, 255, 60},
	// 	{140, 60, 255}, {200, 100, 100}, {100, 200, 100}, {100, 100, 200}
	// };

	// for (size_t i = 0; i < cloudsTransformed.size(); ++i) {
	// 	if (cloudsTransformed[i] && !cloudsTransformed[i]->empty()) {
	// 		auto color = colors[i % colors.size()];
	// 		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(
	// 			cloudsTransformed[i], std::get<0>(color), std::get<1>(color), std::get<2>(color));
	// 		viewer->addPointCloud<pcl::PointXYZ>(cloudsTransformed[i], colorHandler, "cloud" + std::to_string(i));
	// 		viewer->setPointCloudRenderingProperties(
	// 			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud" + std::to_string(i));
	// 	}
	// }

	// // Camera setup
	// if (eyeinhand) {
	// 	viewer->setCameraPosition(0.05, -0.3, 0.28, 0.2, -1, -0.1, 0, 0, 1);
	// } else {
	// 	viewer->setCameraPosition(0.25, -0.25, 0.4, -0.5, 0.5, -0.6, 0, 0, 1);
	// }

	// // Set window size for better screenshot quality
	// viewer->setSize(1920, 1080);

	// // Render multiple times to ensure everything is initialized
	// for (int i = 0; i < 5; ++i) {
	// 	viewer->spinOnce(200);
	// 	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	// }

	// // Save screenshot
	// try {
	// 	std::string screenshot_path = path + "/reghec_result.png";
	// 	viewer->saveScreenshot(screenshot_path);
	// 	std::cout << "Saved screenshot: " << screenshot_path << std::endl;
	// } catch (const std::exception& e) {
	// 	std::cerr << "[error] Screenshot failed: " << e.what() << std::endl;
	// 	std::cerr << "        (Continuing anyway - other outputs are saved)\n";
	// } catch (...) {
	// 	std::cerr << "[error] Screenshot failed with unknown error\n";
	// 	std::cerr << "        (Continuing anyway - other outputs are saved)\n";
	// }

	// // Close viewer immediately (no user interaction needed)
	// viewer->close();

	std::cout << "Processing complete!\n";
	std::cout << "Total views processed: " << cloudsOriginal.size() << std::endl;

	return 0;
}