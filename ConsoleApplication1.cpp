#include "stdafx.h"
#include <iostream>
#include <string>
#include <vector>
//
//����PCL��ʹ��
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
//
//�������Ͷ���PCD
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

//���������ؽ�
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>

//���ڹؼ�����ȡ
#include <pcl/keypoints/sift_keypoint.h>

//��������������PFH
#include <pcl/features/pfh.h>
//����һ����
#include <pcl/registration/ia_ransac.h>

using namespace std;
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//
//�����Ͷ���
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
//PCD�ṹ�嶨�壬�洢�������ݺ�name
struct PCD {
	PointCloud::Ptr cloud;
	string f_name;
	PCD() :cloud(new PointCloud) {};
};

//
//PLYתPCD
void ply2pcd(const string str_1, const string str_2) {
	//"C:/Users/Scott/Desktop/bunny.tar/bunny/data/top3.ply"
	//"C:/Users/Scott/Desktop/bunny.tar/bunny/data/top3.pcd"
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PolygonMesh mesh;
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::loadPolygonFilePLY(str_1, mesh);
	pcl::io::mesh2vtk(mesh, polydata);
	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
	pcl::io::savePCDFileASCII(str_2, *cloud);
}

//
//����PCD�ļ���data����
void loadData(string fileName, vector<PCD, Eigen::aligned_allocator<PCD>> &models) {
	PCD p;
	p.f_name = fileName;
	pcl::io::loadPCDFile(fileName, *p.cloud);
	models.push_back(p);
}

//
//��ȡ�ļ�������
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
//����ICP��׼����
//downsampleΪtrue��ʾ�˲�
void pairAlignClassical(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false) {
	//
	//Ϊ��һ���Ժ͸��ٵ��²���
	//ע�⣺Ϊ�˴����ݼ���Ҫ��������
	//gridΪ�˲��������
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
	cout << "����ICP�㷨:" << endl;
	cout << "The Fitness Score: " << icp.getFitnessScore() << endl;
	cout << "The Number Of Iterations: " << icp.nr_iterations_ << endl;
	cout << "Run time:" << (double)(end - start) / (double)CLOCKS_PER_SEC << "s" << endl;

	Ti = icp.getFinalTransformation();
	//
	// �õ�Ŀ����Ƶ�Դ���Ƶı任
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
	//���Դ���Ƶ�ת��Ŀ��
	*output = *cloud_src + *finalCloud;
	final_transform = targetToSource;
}

//
//ʹ��LM�㷨�Ż����ICP��׼����
//downsampleΪtrue��ʾ�˲�
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
	//
	//Ϊ��һ���Ժ͸��ٵ��²���
	//ע�⣺Ϊ�˴����ݼ���Ҫ��������
	//gridΪ�˲��������
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
	//�������淨��
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	//
	//�������������k
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	//
	//ƴ�ӵ������ݺͷ��߹���
	pcl::copyPointCloud(*src, *points_with_normals_src);
	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);
	//
	// ��׼
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	//
	//��һ��ѭ����������ͬ�����Ż�����ʹ������ӻ�
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);
	//
	//���������ж�����
	reg.setTransformationEpsilon(1e-10);
	//
	//��������Ӧ��ϵ֮���(src<->tgt)����������Ϊ
	//ע�⣺�������ݼ���С������
	reg.setMaxCorrespondenceDistance(150);
	reg.setEuclideanFitnessEpsilon(0.01);
	reg.setMaximumIterations(300);
	
	reg.align(*reg_result);
	clock_t end = clock();
	cout << "LM�Ľ�ICP�㷨:" << endl;
	cout << "The Fitness Score: " << reg.getFitnessScore() << endl;
	cout << "The Number Of Iterations: " << reg.nr_iterations_ << endl;
	cout << "Run time:" << (double)(end - start) / (double)CLOCKS_PER_SEC << "s" << endl;

	/*
	int i = 0;
	while (!reg.hasConverged()) {
		i++;
		reg.align(*reg_result);
	}
	cout << i << endl;
	*/

	Ti = reg.getFinalTransformation();
	//
	// �õ�Ŀ����Ƶ�Դ���Ƶı任
	targetToSource = Ti.inverse();
	//
	//��Ŀ�����ת����Դ���
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
	//���Դ���Ƶ�ת��Ŀ��
	*output += *cloud_src;
	final_transform = targetToSource;
	
}

//
//ƽ������
void smoothingHandle(const PointCloud::Ptr cloud_src) {
	clock_t start = clock();
	PointCloud::Ptr src(new PointCloud);
	src = cloud_src;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// ����ļ�����PointNormal���ͣ������洢�ƶ���С���˷�����ķ���
	pcl::PointCloud<pcl::PointNormal> mls_points;
	// ������� (�ڶ��ֶ���������Ϊ�˴洢����, ��ʹ�ò���Ҳ��Ҫ�������)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	//���ò���
	mls.setInputCloud(src);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.009);

	mls.process(mls_points);
	pcl::io::savePCDFile("smoothingBun0009.pcd", mls_points);
	clock_t end = clock();
	cout << "ƽ������:" << endl;
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
//���������ؽ�
void poiRestruct(const PointCloud::Ptr cloud_src) {
	clock_t start = clock();
	PointCloud::Ptr src(new PointCloud);
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	src = cloud_src;
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	//
	//�������������k
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	//
	//ƴ�ӵ������ݺͷ��߹���
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
	cout << "���������ؽ��㷨:" << endl;
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
//̰��ͶӰ���ǻ�
void restructMethod_2(const PointCloud::Ptr cloud_src) {
	clock_t start = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud = cloud_src;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	tree->setInputCloud(cloud);//��cloud����tree����
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);//���Ʒ��ߴ洢������
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);//�����ֶ�
	//* cloud_with_normals = cloud + normals

	//��������������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);   //���ƹ���������

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//�������ǻ�����
	pcl::PolygonMesh triangles;//�洢�������ǻ�������ģ��

	gp3.setSearchRadius(2.5);//�������ӵ�֮��������룬���������������߳���
	gp3.setMu(2.5);  //���ñ���������������ڵ����Զ����Ϊ2.5��Ϊ��ʹ�õ����ܶȵı仯
	gp3.setMaximumNearestNeighbors(100);    //������������������������
	gp3.setMaximumSurfaceAngle(M_PI / 4); // ����ĳ�㷨�߷���ƫ�������㷨�ߵ����Ƕ�45
	gp3.setMinimumAngle(M_PI / 18); // �������ǻ���õ����������ڽǵ���С�ĽǶ�Ϊ10
	gp3.setMaximumAngle(2*M_PI / 3); // �������ǻ���õ����������ڽǵ����Ƕ�Ϊ120
	gp3.setNormalConsistency(false);  //���øò�����֤���߳���һ��

	gp3.setInputCloud(cloud_with_normals);     //�����������Ϊ�������
	gp3.setSearchMethod(tree2); //����������ʽ
	gp3.reconstruct(triangles); //�ؽ���ȡ���ǻ�

	clock_t end = clock();
	cout << "̰��ͶӰ���ǻ��㷨:" << endl;
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
//�ƶ�������
void restructMethod_3(const PointCloud::Ptr cloud_src) {
	clock_t start = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud = cloud_src;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	tree->setInputCloud(cloud);//��cloud����tree����
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);//���Ʒ��ߴ洢������
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);//�����ֶ�

	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	pcl::MarchingCubes<pcl::PointNormal>::Ptr mc(new pcl::MarchingCubesHoppe<pcl::PointNormal>);
	//����MarchingCubes����Ĳ���
	mc->setIsoLevel(0.0f);
	mc->setGridResolution(30, 30, 30);
	mc->setPercentageExtendGrid(0.0f);
	mc->setSearchMethod(tree2);
	mc->setInputCloud(cloud_with_normals);

	pcl::PolygonMesh mesh;//ִ���ع������������mesh��
	mc->reconstruct(mesh);
	clock_t end = clock();
	cout << "�ƶ��������㷨:" << endl;
	cout << "Run time:" << (double)(end - start) / (double)CLOCKS_PER_SEC << "s" << endl;

	viewer->removePolygonMesh("mesh");
	viewer->removePointCloud("source");
	viewer->addText("restruct result", 10, 10, 0, 0, 0, "v2", v2);
	viewer->addPolygonMesh(mesh, "mesh", v2);
	viewer->spin();
	viewer->removePolygonMesh("mesh");
}

//
//�ؼ���ʹ��
void SIFTKeyPoint(const PointCloud::Ptr cloud_src, PointCloud::Ptr &cloud_key) {
	pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	sift.setInputCloud(cloud_src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	sift.setSearchMethod(tree);
	//sift.setScales(min_scale, n_octaves, n_scales_per_octave);//ָ�������ؼ���ĳ߶ȷ�Χ
	sift.setScales(0.001, 6, 4);
	//sift.setMinimumContrast(min_contrast);//�������ƹؼ��������ֵ
	sift.setMinimumContrast(0.00001);
	sift.compute(result);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(result, *cloud_temp);//��������pcl::PointWithScale������ת��Ϊ������pcl::PointXYZ������
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
//PFH��������
void PFHFeatures(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt) {
	/*
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(cloud_key);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	tree->setInputCloud(cloud_key);//��cloud����tree����
	n.setInputCloud(cloud_key);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);//���Ʒ��ߴ洢������

	pfh.setInputNormals(normals);
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_k;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_k(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	pfh.setSearchMethod(tree_k);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh.setRadiusSearch(0.05);
	pfh.compute(*pfhs);
	cout << pfhs->points.size() << endl;
	*/

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

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n_src;//���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals_src(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src_2(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	tree_src_2->setInputCloud(src);//��cloud����tree����
	n_src.setInputCloud(src);
	n_src.setSearchMethod(tree_src_2);
	n_src.setKSearch(20);
	n_src.compute(*normals_src);//���Ʒ��ߴ洢������
	pfh_src.setInputNormals(normals_src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_k_src(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	pfh_src.setSearchMethod(tree_k_src);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_src(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh_src.setRadiusSearch(0.05);
	pfh_src.compute(*pfhs_src);
	cout << "source keypoints:" << pfhs_src->points.size() << endl;
	//cout << "######" << endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n_tgt;//���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals_tgt(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt_2(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	tree_tgt_2->setInputCloud(tgt);//��cloud����tree����
	n_tgt.setInputCloud(tgt);
	n_tgt.setSearchMethod(tree_tgt_2);
	n_tgt.setKSearch(20);
	n_tgt.compute(*normals_tgt);//���Ʒ��ߴ洢������
	pfh_tgt.setInputNormals(normals_tgt);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_k_tgt(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	pfh_tgt.setSearchMethod(tree_k_tgt);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_tgt(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh_tgt.setRadiusSearch(0.05);
	pfh_tgt.compute(*pfhs_tgt);
	cout << "target keypoints:" << pfhs_tgt->points.size() << endl;
	//cout << "@@@@@@" << endl;

	//SAC��׼
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

	cout << "PFH�㷨:" << endl;
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
	//cout << cloud_src->points.size() << endl << cloud_tgt->points.size() << endl;
	pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift_src;
	pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift_tgt;
	pcl::PointCloud<pcl::PointWithScale> result_src;
	pcl::PointCloud<pcl::PointWithScale> result_tgt;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_key_src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_key_tgt(new pcl::PointCloud<pcl::PointXYZ>);

	sift_src.setInputCloud(cloud_src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src_1(new pcl::search::KdTree<pcl::PointXYZ>());
	sift_src.setSearchMethod(tree_src_1);
	//sift.setScales(min_scale, n_octaves, n_scales_per_octave);//ָ�������ؼ���ĳ߶ȷ�Χ
	sift_src.setScales(0.001, 6, 4);
	//sift.setMinimumContrast(min_contrast);//�������ƹؼ��������ֵ
	sift_src.setMinimumContrast(0.00001);
	sift_src.compute(result_src);
	pcl::copyPointCloud(result_src, *cloud_key_src);//��������pcl::PointWithScale������ת��Ϊ������pcl::PointXYZ������
	//cout << cloud_key_src->points.size() << endl;
	
	sift_tgt.setInputCloud(cloud_tgt);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt_1(new pcl::search::KdTree<pcl::PointXYZ>());
	sift_tgt.setSearchMethod(tree_tgt_1);
	sift_tgt.setScales(0.001, 6, 4);
	sift_tgt.setMinimumContrast(0.00001);
	sift_tgt.compute(result_tgt);
	pcl::copyPointCloud(result_tgt, *cloud_key_tgt);//��������pcl::PointWithScale������ת��Ϊ������pcl::PointXYZ������
	//cout << cloud_key_tgt->points.size() << endl;


	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_src;
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_tgt;
	pfh_src.setInputCloud(cloud_key_src);
	pfh_tgt.setInputCloud(cloud_key_tgt);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n_src;//���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals_src(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src_2(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	tree_src_2->setInputCloud(cloud_key_src);//��cloud����tree����
	n_src.setInputCloud(cloud_key_src);
	n_src.setSearchMethod(tree_src_2);
	n_src.setKSearch(20);
	n_src.compute(*normals_src);//���Ʒ��ߴ洢������
	pfh_src.setInputNormals(normals_src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_k_src(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	pfh_src.setSearchMethod(tree_k_src);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_src(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh_src.setRadiusSearch(0.05);
	pfh_src.compute(*pfhs_src);
	cout << "source keypoints:" << pfhs_src->points.size() << endl;
	//cout << "######" << endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n_tgt;//���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals_tgt(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt_2(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	tree_tgt_2->setInputCloud(cloud_key_tgt);//��cloud����tree����
	n_tgt.setInputCloud(cloud_key_tgt);
	n_tgt.setSearchMethod(tree_tgt_2);
	n_tgt.setKSearch(20);
	n_tgt.compute(*normals_tgt);//���Ʒ��ߴ洢������
	pfh_tgt.setInputNormals(normals_tgt);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_k_tgt(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	pfh_tgt.setSearchMethod(tree_k_tgt);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_tgt(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh_tgt.setRadiusSearch(0.05);
	pfh_tgt.compute(*pfhs_tgt);
	cout << "target keypoints:" << pfhs_tgt->points.size() << endl;
	//cout << "@@@@@@" << endl;

	//SAC��׼
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

	cout << "SIFT����PFH�㷨:" << endl;
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
	cout << "����ѡ��˵����" << endl;
	cout << "##########" << endl;
	cout << "-p1 ����ICP��" << endl;
	cout << "-p2 LM�Ż���ICP��" << endl;
	cout << "-sp1 �²���+PFH����+����׼��" << endl;
	cout << "-sp2 SIFT�ؼ���+PFH����+����׼��" << endl;
	cout << "-s ƽ������" << endl;
	cout << "-q1 ����MC��" << endl;
	cout << "-q2 ���������ؽ���" << endl;
	cout << "-q3 ̰��ͶӰ���ǻ���" << endl;
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
			//����ICP�㷨
			pairAlignClassical(source, target, temp, pairTransform, false);
		}
		else if (strcmp(argv[1], "-p2") == 0) {
			//
			//ʹ��LM�㷨�Ż����ICP��׼
			pairAlign(source, target, temp, pairTransform, false);
		}
		else if (strcmp(argv[1], "-sp1") == 0) {
			//
			//PFH��������
			PFHFeatures(source, target);
		}
		else if (strcmp(argv[1], "-sp2") == 0) {
			//
			//SIFT�ؼ���+PFH��������
			SIFT_PFH(source, target);
		}
		else {
			cout << "������������" << endl;
		}
	}
	if (argc == 3) {
		if (strcmp(argv[1], "-s") == 0) {
			//
			//ƽ������
			smoothingHandle(source);
		}
		else if (strcmp(argv[1], "-q1") == 0) {
			//
			//�ƶ��������㷨
			restructMethod_3(source);
		}
		else if (strcmp(argv[1], "-q2") == 0) {
			//
			//���������ؽ�
			//loadData("C:/Users/Scott/Desktop/bunny.tar/bunny/reconstruction/smoothingBun0009.pcd", bun);
			poiRestruct(source);
		}
		else if (strcmp(argv[1], "-q3") == 0) {
			//
			//̰��ͶӰ���ǻ�
			restructMethod_2(source);
		}
		else {
			cout << "������������" << endl;
		}
	}
	
	//
	//�ؼ���ʹ��,PFH��������
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_key(new pcl::PointCloud<pcl::PointXYZ>);
	//SIFTKeyPoint(bun[0].cloud,cloud_key);
	//cout << cloud_key->points.size() << endl;
	//PFHFeatures(bun[0].cloud, cloud_key);


	/*
	loadData("C:/Users/Scott/Desktop/bunny.tar/bunny/reconstruction/bun_zipper.pcd", bun);
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
	poiRestruct(source);
	*/
	/*
	//loadData("C:/Users/Scott/Desktop/bunny.tar/bunny/reconstruction/bun_zipper.pcd", bun);
	//loadData("C:/Users/Scott/Desktop/bunny.tar/bunny/reconstruction/bun_zipper_t.pcd", bun);
	loadData("C:/Users/Scott/Desktop/bunny.tar/bunny/reconstruction/bun_zipper.pcd", bun);
	loadData("C:/Users/Scott/Desktop/bunny.tar/bunny/reconstruction/bun_zipper_t.pcd", bun);

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

	//pairAlignClassical(source, target, temp, pairTransform, false);
	//pairAlign(source, target, temp, pairTransform, false);
	//PFHFeatures(source, target);
	SIFT_PFH(source, target);*/
	

	/*
	loadData("C:/Users/Scott/Desktop/bunny.tar/bunny/glo/glo.pcd", bun);

	PointCloud::Ptr src(new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	grid.setLeafSize(0.8, 0.8, 0.8);
	source = bun[0].cloud;
	grid.setInputCloud(source);
	grid.filter(*src);
	pcl::io::savePCDFileASCII("C:/Users/Scott/Desktop/bunny.tar/bunny/glo/gloo.pcd", *src);
	
	//int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(255, 255, 255, v1);
	//int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(255, 255, 255, v2);
	viewer->addText("source pointclod dataset(red)", 10, 10, 0, 0, 0, "v1", v1);
	viewer->removePointCloud("v1_source");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_color(src, 255, 0, 0);
	viewer->addPointCloud(src, src_color, "v1_source", v1);
	PCL_INFO("Press q to begin the registration.\n");
	viewer->spin();
	*/

	//restructMethod_3(source);
	//smoothingHandle(source);
	//poiRestruct(source);
	//restructMethod_2(src);
	
	

	return 0;
}
