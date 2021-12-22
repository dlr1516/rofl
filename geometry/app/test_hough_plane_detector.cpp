#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <memory>

#include <rofl/geometry/hough_plane_detector.h>
#include <rofl/common/param_map.h>
#include <rofl/common/profiler.h>
#include <rofl/common/macros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;

void plotGrid(const rofl::HoughPlaneDetector& hpd, const std::string& filename);

void plotPlane(const typename rofl::HoughPlaneDetector::PlaneParam& params, const PointCloud& cloud, float distMax, pcl::ModelCoefficients& coefficients,
		float& x, float& y, float& z, int& n);

int main(int argc, char** argv) {
	typename PointCloud::Ptr cloudIn(new PointCloud);
	typename PointCloud::Ptr cloudKeypoint(new PointCloud);
	typename PointCloud::Ptr cloudPlane(new PointCloud);
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	rofl::ParamMap params;
	rofl::HoughPlaneDetector hpd;
	typename rofl::HoughPlaneDetector::VectorPlaneParam planes;
	std::string filenameIn, filenameCfg;
	int thetaNum, phiNum, rhoNum;
	rofl::Scalar thetaWin, phiWin, rhoWin, rhoStep;
	int pointSize;
	bool enablePlot;
	PointType normalP1, normalP2;

	params.read(argc, argv);
	params.getParam<std::string>("cfg", filenameCfg, std::string(""));
	if (!params.read(filenameCfg)) {
		std::cout << "Cannot open configuration file \"" << filenameCfg << "\": using default values" << std::endl;
	}
	params.read(argc, argv);

	// Read params
	params.getParam<std::string>("in", filenameIn, "");
	params.getParam("pointSize", pointSize, 2);
	params.getParam<int>("thetaNum", thetaNum, 90);
	params.getParam<int>("phiNum", phiNum, 360);
	params.getParam<int>("rhoNum", rhoNum, 240);
	params.getParam<float>("rhoStep", rhoStep, 0.025);
	params.getParam<rofl::Scalar>("thetaWin", thetaWin, rofl::Scalar(5.0));
	params.getParam<rofl::Scalar>("phiWin", phiWin, rofl::Scalar(5.0));
	params.getParam<rofl::Scalar>("rhoWin", rhoWin, rofl::Scalar(5 * rhoStep));
	thetaWin *= M_PI / 180.0;
	phiWin *= M_PI / 180.0;
	params.getParam<bool>("enablePlot", enablePlot, true);

	std::cout << "\nParams:\n";
	params.write(std::cout);
	std::cout << std::endl;

	ROFL_VAR1(__cplusplus);

	hpd.init(thetaNum, phiNum, rhoNum, rhoStep);
	hpd.setPeakWindow(thetaWin, phiWin, rhoWin);

	std::cout << "Loading point cloud src from \"" << filenameIn << "\"" << std::endl;
	if (pcl::io::loadPCDFile(filenameIn, *cloudIn) < 0) {
		std::cerr << "Cannot load point cloud from \"" << filenameIn << "\"" << std::endl;
		return -1;
	}
	std::cout << "input cloud has " << cloudIn->size() << " points" << std::endl;

	// Sets the parameters to detect SKIP edges
	//	cloudIn->width = cloudWidth;
	//	cloudIn->height = cloudHeight;

	auto pclToEigen = [](const PointType& point) -> rofl::Vector3 {
		rofl::Vector3 pointEig(point.x, point.y, point.z);
		return pointEig;
	};
	std::cout << "inserting and computing HT and HS" << std::endl;
	{
		rofl::ScopedTimer timer("hpd::insert()");
		hpd.insert(cloudIn->begin(), cloudIn->end(), pclToEigen);
	}
	std::cout << "\n---\nprofiler:" << std::endl;
	rofl::Profiler::getProfiler().printStats(std::cout);

	std::cout << "saving Hough spectrum grid on Gnuplot file" << std::endl;
	plotGrid(hpd, "hough_spectrum_cloud.plot");

	std::cout << "finding best directions:" << std::endl;
	std::vector<rofl::HoughPlaneDetector::Indices2> hsMaxima;
	hpd.findSpectrumMax(hsMaxima);
	for (auto &hmax : hsMaxima) {
		std::cout
		<< "  [" << hmax[0] << "," << hmax[1] << "]: normal [" << hpd.getNormal(hmax).transpose() << "]" << " HS " << hpd.getHoughSpectrum(hmax)
		<< std::endl;
	}

//	std::cout << "finding planes" << std::endl;
//	hpd.findPlanes(planes);
	if (!hsMaxima.empty()) {
		std::cout << "find parallel planes to maximum normal:" << std::endl;
		int ithetaMax = hsMaxima[0][0];
		int iphiMax = hsMaxima[0][1];
		hpd.findParallelPlanes(ithetaMax, iphiMax, planes);
		for (int i = 0; i < planes.size(); ++i) {
			std::cout << "planes[" << i << "] = [" << planes[i].transpose() << "]\n";
		}
	}
	else {
		std::cout << "NO maximum normal" << std::endl;
	}

	viewer = std::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Box Detector"));
	viewer->setBackgroundColor(0.9, 0.9, 0.9);
	//viewer->setCameraPosition(1.0, 1.0, fabs(2.0 * aligner.getPlaneCoeffs()(3)), 0.0, 0.0, -1.0);
	pcl::visualization::PointCloudColorHandlerCustom<PointType> cloudInHander(cloudIn, 0, 127, 127);
	viewer->addPointCloud<PointType>(cloudIn, cloudInHander, "cloudIn");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloudIn");

	if (!hsMaxima.empty()) {
		normalP1.x = normalP1.y = normalP1.z = 0.0f;
		normalP2.x = hpd.getNormal(hsMaxima[0])(0);
		normalP2.y = hpd.getNormal(hsMaxima[0])(1);
		normalP2.z = hpd.getNormal(hsMaxima[0])(2);
		viewer->addArrow(normalP2, normalP1, 1.0f, 1.0f, 0.0, true, "normalMax");
	}

	for (int i = 0; i < planes.size(); ++i) {
		pcl::ModelCoefficients coefficients;
		float x, y, z;
		int n;
		std::stringstream ss;

		plotPlane(planes[i], *cloudIn, 2.0 * rhoStep, coefficients, x, y, z, n);

		ss << "plane_" << i;
		std::cout << "  " << ss.str() << " [" << planes[i].transpose() << "]  centered in [" << x << "," << y << "," << z << "]  inliers " << n << std::endl;

		if (n > 100) {
			viewer->addPlane(coefficients, x, y, z, ss.str());
			PointType textPos;
			textPos.x = x + 0.10 * planes[i](0);
			textPos.y = y + 0.10 * planes[i](1);
			textPos.z = z + 0.10 * planes[i](2);
			ss.str("");
			ss << "pl_" << i;
			viewer->addText3D(ss.str(), textPos, 0.05, 0.0, 0.0, 0.0);
		}
	}

	viewer->addCoordinateSystem(0.5);
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) { // && !plotter.wasStopped()) {
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	return 0;
}

void plotGrid(const rofl::HoughPlaneDetector& hpd, const std::string& filename) {
	float thetaRes, phiRes, thetaMin, phiMin;

	std::ofstream file(filename);
	if (!file) {
		std::cerr << "Cannot open \"" << filename << "\"" << std::endl;
		return;
	}

	file << "set grid nopolar\n" << "set style data lines\n" << "set dgrid3d " << hpd.getThetaNum() << "," << hpd.getPhiNum() << "\n" << "set hidden3d\n";
	file << "splot '-'\n";
	thetaRes = 180.0 / M_PI * hpd.getThetaRes();
	phiRes = 180.0 / M_PI * hpd.getPhiRes();
	thetaMin = 180.0 / M_PI * hpd.getThetaMin();
	phiMin = 180.0 / M_PI * hpd.getPhiMin();
	for (int itheta = 0; itheta < hpd.getThetaNum(); ++itheta) {
		for (int iphi = 0; iphi < hpd.getPhiNum(); ++iphi) {
			file << (thetaRes * itheta + thetaMin) << " " << (phiRes * iphi + phiMin) << " " << hpd.getHoughSpectrum(itheta, iphi) << "\n";
		}
	}
	file << "e\n";
	file.close();
}

void plotPlane(const typename rofl::HoughPlaneDetector::PlaneParam& params, const PointCloud& cloud, float distMax, pcl::ModelCoefficients& coefficients,
		float& x, float& y, float& z, int& n) {
	coefficients.values.resize(4);
	coefficients.values[0] = params(0);
	coefficients.values[1] = params(1);
	coefficients.values[2] = params(2);
	coefficients.values[3] = params(3);

	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
	n = 0;
	for (auto &p : cloud.points) {
		float dist = fabs(p.x * params(0) + p.y * params(1) + p.z * params(2) + params(3));
		if (dist < distMax) {
			x += p.x;
			y += p.y;
			z += p.z;
			n++;
		}
	}
	if (n > 0) {
		x = x / n;
		y = y / n;
		z = z / n;
	}
}

