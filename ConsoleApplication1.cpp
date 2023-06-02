#include "stdafx.h"
#include <iostream>
#include <string>
#include <vector>
//
//基本PCL库使用
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
//
//用于类型定义PCD
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/time.h>
#include <pcl/common/io.h>
#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

//用于曲面重建
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>

//用于关键点提取
#include <pcl/keypoints/sift_keypoint.h>

//用于特征描述的PFH
#include <pcl/features/pfh.h>
//采样一致性
#include <pcl/registration/ia_ransac.h>

using namespace std;
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//
//简单类型定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

boost::shared_ptr<pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("show"));
int v1(0);
int v2(0);

namespace pcl
{
	template<>
	struct SIFTKeypointFieldSelector<PointXYZ>
	{
		inline float
			operator () (const PointXYZ &p) const
		{
			return p.z;
		}
	};
}

//
//PCD结构体定义，存储点云数据和name
struct PCD {
	PointCloud::Ptr cloud;
	string f_name;
	PCD() :cloud(new PointCloud) {};
};

//
//PLY转PCD
void ply2pcd(const string str_1, const string str_2) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PolygonMesh mesh;
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::loadPolygonFilePLY(str_1, mesh);
	pcl::io::mesh2vtk(mesh, polydata);
	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
	pcl::io::savePCDFileASCII(str_2, *cloud);
}

//
//加载PCD文件到data容器
void loadData(string fileName, vector<PCD, Eigen::aligned_allocator<PCD>> &models) {
	PCD p;
	p.f_name = fileName;
	pcl::io::loadPCDFile(fileName, *p.cloud);
	models.push_back(p);
}

//
//获取文件名函数
void getFiles(string dirName,vector<string> &filesVec) {
	_finddata_t file;
	long long lf;
	string p;
	if ((lf = _findfirst(p.assign(dirName).append("/*.pcd").c_str(), &file)) == -1) {
		cout << "ERROR!" << endl;
	}
	else {
		//cout << file.name << endl;
		filesVec.push_back(file.name);
		while (_findnext(lf, &file) == 0) {
			//cout << file.name << endl;
			filesVec.push_back(file.name);
		}
	}
	_findclose(lf);
}

//
//经典ICP配准函数
//downsample为true表示滤波
void pairAlignClassical(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false) {
	//
	//为了一致性和高速的下采样
	//注意：为了大数据集需要允许这项
	//grid为滤波处理对象
	clock_t start = clock();
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	if (downsample)
	{
		grid.setLeafSize(0.05, 0.05, 0.05);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}
	pcl::IterativeClosestPoint < pcl::PointXYZ, pcl::PointXYZ> icp;
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), targetToSource;
	icp.setInputSource(src);
	icp.setInputTarget(tgt);
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
	icp.setTransformationEpsilon(1e-10);
	icp.setMaxCorrespondenceDistance(150);
	icp.setEuclideanFitnessEpsilon(0.01);
	icp.setMaximumIterations(300);
	icp.align(*finalCloud);
	clock_t end = clock();
	cout << "经典ICP算法:" << endl;
	cout << "The Fitness Score: " << icp.getFitnessScore() << endl;
	cout << "The Number Of Iterations: " << icp.nr_iterations_ << endl;
	cout << "Run time:" << (double)(end - start) / (double)CLOCKS_PER_SEC << "s" << endl;

	Ti = icp.getFinalTransformation();
	//
	// 得到目标点云到源点云的变换
	targetToSource = Ti.inverse();
	pcl::transformPointCloud(*cloud_tgt, *finalCloud, targetToSource);
	viewer->removePointCloud("source");
	viewer->removePointCloud("target");
	viewer->removeText3D("v2", v2);
	viewer->addText("source pointclod dataset(red), output pointcloud dataset(blue)", 10, 10, 0, 0, 0, "v2", v2);
	PointCloudColorHandlerCustom<PointT> cloud_tgt_h(finalCloud, 0, 0, 255);
	PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	viewer->addPointCloud(finalCloud, cloud_tgt_h, "target", v2);
	viewer->addPointCloud(cloud_src, cloud_src_h, "source", v2);
	PCL_INFO("Press q to continue the registration.\n");
	viewer->spin();
	viewer->removePointCloud("source");
	viewer->removePointCloud("target");
	//
	//添加源点云到转换目标
	*output = *cloud_src + *finalCloud;
	final_transform = targetToSource;
}

//
//使用LM算法优化后的ICP配准函数
//downsample为true表示滤波
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
	//
	//为了一致性和高速的下采样
	//注意：为了大数据集需要允许这项
	//grid为滤波处理对象
	clock_t start = clock();
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	if (downsample)
	{
		grid.setLeafSize(0.005, 0.005, 0.005);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}
	//
	//计算曲面法线
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	//
	//设置搜索对象和k
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	//
	//拼接点云数据和法线估计
	pcl::copyPointCloud(*src, *points_with_normals_src);
	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);
	//
	// 配准
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	//
	//在一个循环中运行相同的最优化并且使结果可视化
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);
	//
	//设置收敛判断条件
	reg.setTransformationEpsilon(1e-10);
	//
	//将两个对应关系之间的(src<->tgt)最大距离设置为
	//注意：根据数据集大小来调整
	reg.setMaxCorrespondenceDistance(150);
	reg.setEuclideanFitnessEpsilon(0.01);
	reg.setMaximumIterations(300);
	
	reg.align(*reg_result);
	clock_t end = clock();
	cout << "LM改进ICP算法:" << endl;
	cout << "The Fitness Score: " << reg.getFitnessScore() << endl;
	cout << "The Number Of Iterations: " << reg.nr_iterations_ << endl;
	cout << "Run time:" << (double)(end - start) / (double)CLOCKS_PER_SEC << "s" << endl;

	Ti = reg.getFinalTransformation();
	//
	// 得到目标点云到源点云的变换
	targetToSource = Ti.inverse();
	//
	//把目标点云转换回源框架
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
	viewer->removePointCloud("source");
	viewer->removePointCloud("target");
	viewer->addText("source pointclod dataset(red), output pointcloud dataset(blue)", 10, 10, 0, 0, 0, "v2", v2);
	PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 0, 255);
	PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	viewer->addPointCloud(output, cloud_tgt_h, "target", v2);
	viewer->addPointCloud(cloud_src, cloud_src_h, "source", v2);
	PCL_INFO("Press q to continue the registration.\n");
	viewer->spin();
	viewer->removePointCloud("source");
	viewer->removePointCloud("target");
	//
	//添加源点云到转换目标
	*output += *cloud_src;
	final_transform = targetToSource;
	
}

//
//平滑处理
void smoothingHandle(const PointCloud::Ptr cloud_src) {
	clock_t start = clock();
	PointCloud::Ptr src(new PointCloud);
	src = cloud_src;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// 输出文件中有PointNormal类型，用来存储移动最小二乘法算出的法线
	pcl::PointCloud<pcl::PointNormal> mls_points;
	// 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	//设置参数
	mls.setInputCloud(src);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.009);

	mls.process(mls_points);
	pcl::io::savePCDFile("smoothingBun0009.pcd", mls_points);
	clock_t end = clock();
	cout << "平滑处理:" << endl;
	cout << "Run time:" << (double)(end - start) / (double)CLOCKS_PER_SEC << "s" << endl;

	PointCloud::Ptr cloud(new PointCloud);
	pcl::io::loadPCDFile("smoothingBun.pcd", *cloud);
	viewer->removePointCloud("source");
	viewer->addText("smoothing result", 10, 10, 0, 0, 0, "v2", v2);
	PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud, 255, 0, 0);
	viewer->addPointCloud(cloud, cloud_src_h, "source", v2);
	PCL_INFO("Press q to continue the registration.\n");
	viewer->spin();
	viewer->removePointCloud("source");

}

//
//泊松曲面重建
void poiRestruct(const PointCloud::Ptr cloud_src) {
	clock_t start = clock();
	PointCloud::Ptr src(new PointCloud);
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	src = cloud_src;
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	//
	//设置搜索对象和k
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	//
	//拼接点云数据和法线估计
	pcl::copyPointCloud(*src, *points_with_normals_src);
	
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree_2(new pcl::search::KdTree<pcl::PointNormal>());
	tree_2->setInputCloud(points_with_normals_src);
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setInputCloud(points_with_normals_src);
	poisson.setSearchMethod(tree_2);
	//poisson.setDegree(2);
	//poisson.setConfidence(false);
	//poisson.setManifold(true);
	//poisson.setOutputPolygons(false);
	poisson.setDepth(12);
	//poisson.setSolverDivide(7);
	//poisson.setSamplesPerNode(5);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);
	clock_t end = clock();
	cout << "泊松曲面重建算法:" << endl;
	cout << "Run time:" << (double)(end - start) / (double)CLOCKS_PER_SEC << "s" << endl;

	pcl::io::savePLYFile("this.ply", mesh);

	viewer->removePolygonMesh("mesh");
	viewer->removePointCloud("source");
	viewer->addText("restruct result", 10, 10, 0, 0, 0, "v2", v2);
	viewer->addPolygonMesh(mesh, "mesh", v2);
	viewer->spin();
	viewer->removePolygonMesh("mesh");
}

//
//贪婪投影三角化
void restructMethod_2(const PointCloud::Ptr cloud_src) {
	clock_t start = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud = cloud_src;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	tree->setInputCloud(cloud);//用cloud构建tree对象
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);//估计法线存储到其中
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);//连接字段
	//* cloud_with_normals = cloud + normals

	//定义搜索树对象
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);   //点云构建搜索树

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
	pcl::PolygonMesh triangles;//存储最终三角化的网络模型

	gp3.setSearchRadius(2.5);//设置连接点之间的最大距离，（即是三角形最大边长）
	gp3.setMu(2.5);  //设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
	gp3.setMaximumNearestNeighbors(100);    //设置样本点可搜索的邻域个数
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度45
	gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到的三角形内角的最小的角度为10
	gp3.setMaximumAngle(2*M_PI / 3); // 设置三角化后得到的三角形内角的最大角度为120
	gp3.setNormalConsistency(false);  //设置该参数保证法线朝向一致

	gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
	gp3.setSearchMethod(tree2); //设置搜索方式
	gp3.reconstruct(triangles); //重建提取三角化

	clock_t end = clock();
	cout << "贪婪投影三角化算法:" << endl;
	cout << "Run time:" << (double)(end - start) / (double)CLOCKS_PER_SEC << "s" << endl;

	//vector<int> parts = gp3.getPartIDs();
	//vector<int> states = gp3.getPointStates();

	viewer->removePolygonMesh("mesh");
	viewer->removePointCloud("source");
	viewer->addText("restruct result", 10, 10, 0, 0, 0, "v2", v2);
	viewer->addPolygonMesh(triangles, "mesh", v2);
	viewer->spin();
	viewer->removePolygonMesh("mesh");

}

//
//移动立方体
void restructMethod_3(const PointCloud::Ptr cloud_src) {
	clock_t start = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud = cloud_src;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	tree->setInputCloud(cloud);//用cloud构建tree对象
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);//估计法线存储到其中
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);//连接字段

	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	pcl::MarchingCubes<pcl::PointNormal>::Ptr mc(new pcl::MarchingCubesHoppe<pcl::PointNormal>);
	//设置MarchingCubes对象的参数
	mc->setIsoLevel(0.0f);
	mc->setGridResolution(30, 30, 30);
	mc->setPercentageExtendGrid(0.0f);
	mc->setSearchMethod(tree2);
	mc->setInputCloud(cloud_with_normals);

	pcl::PolygonMesh mesh;//执行重构，结果保存在mesh中
	mc->reconstruct(mesh);
	clock_t end = clock();
	cout << "移动立方体算法:" << endl;
	cout << "Run time:" << (double)(end - start) / (double)CLOCKS_PER_SEC << "s" << endl;

	viewer->removePolygonMesh("mesh");
	viewer->removePointCloud("source");
	viewer->addText("restruct result", 10, 10, 0, 0, 0, "v2", v2);
	viewer->addPolygonMesh(mesh, "mesh", v2);
	viewer->spin();
	viewer->removePolygonMesh("mesh");
}

//
//关键点使用
void SIFTKeyPoint(const PointCloud::Ptr cloud_src, PointCloud::Ptr &cloud_key) {
	pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	sift.setInputCloud(cloud_src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	sift.setSearchMethod(tree);
	//sift.setScales(min_scale, n_octaves, n_scales_per_octave);//指定搜索关键点的尺度范围
	sift.setScales(0.001, 6, 4);
	//sift.setMinimumContrast(min_contrast);//设置限制关键点检测的阈值
	sift.setMinimumContrast(0.00001);
	sift.compute(result);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(result, *cloud_temp);//将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据
	pcl::copyPointCloud(result, *cloud_key);
	cout << cloud_temp->size() << endl;
	cout << cloud_temp->points.size() << endl;

	viewer->removePointCloud("source");
	viewer->removePointCloud("sift");
	viewer->addText("source pointclod dataset(red), SIFT Keypointcloud dataset(blue)", 10, 10, 0, 0, 0, "v2", v2);
	PointCloudColorHandlerCustom<PointT> cloud_tgt_h(cloud_temp, 0, 0, 255);
	PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	viewer->addPointCloud(cloud_temp, cloud_tgt_h, "sift", v2);
	viewer->addPointCloud(cloud_src, cloud_src_h, "source", v2);
	PCL_INFO("Press q to continue the registration.\n");
	viewer->spin();
	viewer->removePointCloud("source");
	viewer->removePointCloud("sift");

}

//
//PFH特征描述
void PFHFeatures(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt) {

	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	grid.setLeafSize(0.0062, 0.0062, 0.0062);
	grid.setInputCloud(cloud_src);
	grid.filter(*src);
	grid.setInputCloud(cloud_tgt);
	grid.filter(*tgt);
	clock_t start = clock();
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_src;
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_tgt;
	pfh_src.setInputCloud(src);
	pfh_tgt.setInputCloud(tgt);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n_src;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals_src(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src_2(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	tree_src_2->setInputCloud(src);//用cloud构建tree对象
	n_src.setInputCloud(src);
	n_src.setSearchMethod(tree_src_2);
	n_src.setKSearch(20);
	n_src.compute(*normals_src);//估计法线存储到其中
	pfh_src.setInputNormals(normals_src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_k_src(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	pfh_src.setSearchMethod(tree_k_src);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_src(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh_src.setRadiusSearch(0.05);
	pfh_src.compute(*pfhs_src);
	cout << "source keypoints:" << pfhs_src->points.size() << endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n_tgt;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals_tgt(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt_2(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	tree_tgt_2->setInputCloud(tgt);//用cloud构建tree对象
	n_tgt.setInputCloud(tgt);
	n_tgt.setSearchMethod(tree_tgt_2);
	n_tgt.setKSearch(20);
	n_tgt.compute(*normals_tgt);//估计法线存储到其中
	pfh_tgt.setInputNormals(normals_tgt);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_k_tgt(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	pfh_tgt.setSearchMethod(tree_k_tgt);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_tgt(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh_tgt.setRadiusSearch(0.05);
	pfh_tgt.compute(*pfhs_tgt);
	cout << "target keypoints:" << pfhs_tgt->points.size() << endl;

	//SAC配准
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::PFHSignature125> scia;
	scia.setInputSource(src);
	scia.setInputTarget(tgt);
	scia.setSourceFeatures(pfhs_src);
	scia.setTargetFeatures(pfhs_tgt);
	//scia.setMinSampleDistance(1);
	//scia.setNumberOfSamples(2);
	//scia.setCorrespondenceRandomness(20);
	PointCloud::Ptr sac_result(new PointCloud);
	scia.align(*sac_result);
	clock_t end = clock();

	cout << "PFH算法:" << endl;
	cout << "The Fitness Score: " << scia.getFitnessScore() << endl;
	cout << "Run time:" << (double)(end - start) / (double)CLOCKS_PER_SEC << "s" << endl;


	Eigen::Matrix4f sac_trans, sac_trans_t;
	sac_trans = scia.getFinalTransformation();
	sac_trans_t = sac_trans.inverse();
	pcl::transformPointCloud(*cloud_tgt, *sac_result, sac_trans_t);

	viewer->removePointCloud("source");
	viewer->removePointCloud("pfh");
	viewer->addText("source pointclod dataset(red), PFH (blue)", 10, 10, 0, 0, 0, "v2", v2);
	PointCloudColorHandlerCustom<PointT> cloud_src_h(src, 255, 0, 0);
	PointCloudColorHandlerCustom<PointT> cloud_tgt_h(sac_result, 0, 0, 255);
	viewer->addPointCloud(src, cloud_src_h, "source", v2);
	viewer->addPointCloud(sac_result, cloud_tgt_h, "pfh", v2);
	PCL_INFO("Press q to continue the registration.\n");
	viewer->spin();
	viewer->removePointCloud("source");
	viewer->removePointCloud("pfh");

	PointCloud::Ptr temp(new PointCloud);
	Eigen::Matrix4f pairTransform;
	pairAlignClassical(cloud_src, sac_result, temp, pairTransform, false);

}

//
//
void SIFT_PFH(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt) {
	clock_t start = clock();
	pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift_src;
	pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift_tgt;
	pcl::PointCloud<pcl::PointWithScale> result_src;
	pcl::PointCloud<pcl::PointWithScale> result_tgt;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_key_src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_key_tgt(new pcl::PointCloud<pcl::PointXYZ>);

	sift_src.setInputCloud(cloud_src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src_1(new pcl::search::KdTree<pcl::PointXYZ>());
	sift_src.setSearchMethod(tree_src_1);
	//sift.setScales(min_scale, n_octaves, n_scales_per_octave);//指定搜索关键点的尺度范围
	sift_src.setScales(0.001, 6, 4);
	//sift.setMinimumContrast(min_contrast);//设置限制关键点检测的阈值
	sift_src.setMinimumContrast(0.00001);
	sift_src.compute(result_src);
	pcl::copyPointCloud(result_src, *cloud_key_src);//将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据
	
	sift_tgt.setInputCloud(cloud_tgt);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt_1(new pcl::search::KdTree<pcl::PointXYZ>());
	sift_tgt.setSearchMethod(tree_tgt_1);
	sift_tgt.setScales(0.001, 6, 4);
	sift_tgt.setMinimumContrast(0.00001);
	sift_tgt.compute(result_tgt);
	pcl::copyPointCloud(result_tgt, *cloud_key_tgt);//将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据


	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_src;
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_tgt;
	pfh_src.setInputCloud(cloud_key_src);
	pfh_tgt.setInputCloud(cloud_key_tgt);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n_src;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals_src(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src_2(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	tree_src_2->setInputCloud(cloud_key_src);//用cloud构建tree对象
	n_src.setInputCloud(cloud_key_src);
	n_src.setSearchMethod(tree_src_2);
	n_src.setKSearch(20);
	n_src.compute(*normals_src);//估计法线存储到其中
	pfh_src.setInputNormals(normals_src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_k_src(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	pfh_src.setSearchMethod(tree_k_src);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_src(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh_src.setRadiusSearch(0.05);
	pfh_src.compute(*pfhs_src);
	cout << "source keypoints:" << pfhs_src->points.size() << endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n_tgt;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals_tgt(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt_2(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	tree_tgt_2->setInputCloud(cloud_key_tgt);//用cloud构建tree对象
	n_tgt.setInputCloud(cloud_key_tgt);
	n_tgt.setSearchMethod(tree_tgt_2);
	n_tgt.setKSearch(20);
	n_tgt.compute(*normals_tgt);//估计法线存储到其中
	pfh_tgt.setInputNormals(normals_tgt);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_k_tgt(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	pfh_tgt.setSearchMethod(tree_k_tgt);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_tgt(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh_tgt.setRadiusSearch(0.05);
	pfh_tgt.compute(*pfhs_tgt);
	cout << "target keypoints:" << pfhs_tgt->points.size() << endl;

	//SAC配准
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::PFHSignature125> scia;
	scia.setInputSource(cloud_key_src);
	scia.setInputTarget(cloud_key_tgt);
	scia.setSourceFeatures(pfhs_src);
	scia.setTargetFeatures(pfhs_tgt);
	//scia.setMinSampleDistance(1);
	//scia.setNumberOfSamples(2);
	//scia.setCorrespondenceRandomness(20);
	PointCloud::Ptr sac_result(new PointCloud);
	scia.align(*sac_result);
	clock_t end = clock();

	cout << "SIFT——PFH算法:" << endl;
	cout << "The Fitness Score: " << scia.getFitnessScore() << endl;
	cout << "Run time:" << (double)(end - start) / (double)CLOCKS_PER_SEC << "s" << endl;


	Eigen::Matrix4f sac_trans, sac_trans_t;
	sac_trans = scia.getFinalTransformation();
	sac_trans_t = sac_trans.inverse();
	pcl::transformPointCloud(*cloud_tgt, *sac_result, sac_trans_t);

	viewer->removePointCloud("source");
	viewer->removePointCloud("pfh");
	viewer->addText("source pointclod dataset(red), SIFT PFH (blue)", 10, 10, 0, 0, 0, "v2", v2);
	PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	PointCloudColorHandlerCustom<PointT> cloud_tgt_h(sac_result, 0, 0, 255);
	viewer->addPointCloud(cloud_src, cloud_src_h, "source", v2);
	viewer->addPointCloud(sac_result, cloud_tgt_h, "pfh", v2);
	PCL_INFO("Press q to continue the registration.\n");
	viewer->spin();
	viewer->removePointCloud("source");
	viewer->removePointCloud("pfh");

	PointCloud::Ptr temp(new PointCloud);
	Eigen::Matrix4f pairTransform;
	pairAlignClassical(cloud_src, sac_result, temp, pairTransform, false);
}

int main(int argc, char** argv)
{
	cout << "功能选择说明：" << endl;
	cout << "##########" << endl;
	cout << "-p1 经典ICP：" << endl;
	cout << "-p2 LM优化的ICP：" << endl;
	cout << "-sp1 下采样+PFH特征+粗配准：" << endl;
	cout << "-sp2 SIFT关键点+PFH特征+粗配准：" << endl;
	cout << "-s 平滑处理：" << endl;
	cout << "-q1 经典MC：" << endl;
	cout << "-q2 泊松曲面重建：" << endl;
	cout << "-q3 贪婪投影三角化：" << endl;
	vector<PCD, Eigen::aligned_allocator<PCD>> bun;
	string str_1, str_2;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
	PointCloud::Ptr temp(new PointCloud), source, target;
	if (argc == 3) {
		str_1 = argv[2];
		loadData(str_1, bun);
		//int v1(0);
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->setBackgroundColor(255, 255, 255, v1);
		//int v2(0);
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer->setBackgroundColor(255, 255, 255, v2);
		viewer->addText("source pointclod dataset(red)", 10, 10, 0, 0, 0, "v1", v1);
		viewer->removePointCloud("v1_source");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_color(bun[0].cloud, 255, 0, 0);
		viewer->addPointCloud(bun[0].cloud, src_color, "v1_source", v1);
		PCL_INFO("Press q to begin the registration.\n");
		viewer->spin();
		
		source = bun[0].cloud;
	}
	else if (argc == 4) {
		str_1 = argv[2];
		str_2 = argv[3];
		//loadData("C:/Users/Scott/Desktop/bunny.tar/bunny/reconstruction/bun_zipper.pcd", bun);
		//loadData("C:/Users/Scott/Desktop/bunny.tar/bunny/reconstruction/bun_zipper_t.pcd", bun);
		loadData(str_1, bun);
		loadData(str_2, bun);

		//int v1(0);
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->setBackgroundColor(255, 255, 255, v1);
		viewer->addText("source pointclod dataset(red), target pointcloud dataset(green)", 10, 10, 0, 0, 0, "v1", v1);
		//int v2(0);
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer->setBackgroundColor(255, 255, 255, v2);
		viewer->removePointCloud("v1_target");
		viewer->removePointCloud("v1_source");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_color(bun[0].cloud, 255, 0, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_color(bun[1].cloud, 0, 255, 0);
		viewer->addPointCloud(bun[0].cloud, src_color, "v1_source", v1);
		viewer->addPointCloud(bun[1].cloud, tgt_color, "v1_target", v1);
		PCL_INFO("Press q to begin the registration.\n");
		viewer->spin();
	
		source = bun[0].cloud;
		target = bun[1].cloud;
	}
	
	if (argc == 4) {
		if (strcmp(argv[1], "-p1") == 0) {
			//
			//经典ICP算法
			pairAlignClassical(source, target, temp, pairTransform, false);
		}
		else if (strcmp(argv[1], "-p2") == 0) {
			//
			//使用LM算法优化后的ICP配准
			pairAlign(source, target, temp, pairTransform, false);
		}
		else if (strcmp(argv[1], "-sp1") == 0) {
			//
			//PFH特征描述
			PFHFeatures(source, target);
		}
		else if (strcmp(argv[1], "-sp2") == 0) {
			//
			//SIFT关键点+PFH特征描述
			SIFT_PFH(source, target);
		}
		else {
			cout << "参数输入有误！" << endl;
		}
	}
	if (argc == 3) {
		if (strcmp(argv[1], "-s") == 0) {
			//
			//平滑处理
			smoothingHandle(source);
		}
		else if (strcmp(argv[1], "-q1") == 0) {
			//
			//移动立方体算法
			restructMethod_3(source);
		}
		else if (strcmp(argv[1], "-q2") == 0) {
			//
			//泊松曲面重建
			poiRestruct(source);
		}
		else if (strcmp(argv[1], "-q3") == 0) {
			//
			//贪婪投影三角化
			restructMethod_2(source);
		}
		else {
			cout << "参数输入有误！" << endl;
		}
	}
	return 0;
}
