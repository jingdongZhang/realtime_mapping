#include <sstream>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <vector>
#include <string>
#include <math.h>
#include <inttypes.h>
#include <limits>       // std::numeric_limits

#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>  // For Normal Estimation
#include <pcl/common/common.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>  // For Normal Estimation
#include <pcl/features/integral_image_normal.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/ndt.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/stuff/sampler.h>
#include <g2o/stuff/command_args.h>
#include <g2o/core/factory.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
//#include "ndt_2d.h"

typedef pcl::PointXYZ t_XYZ;
typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

int user_data;

/*
void
  GetAllFiles( std::string path, std::vector<std::string>& files)
{

  long   hFile   =   0;
  //�ļ���Ϣ
  struct _finddata_t fileinfo;
  std::string p;
  if((hFile = _findfirst(p.assign(path).append("\\*").c_str(),&fileinfo)) !=  -1)
  {
    do
    {
      if((fileinfo.attrib &  _A_SUBDIR))
      {
        if(strcmp(fileinfo.name,".") != 0  &&  strcmp(fileinfo.name,"..") != 0)
        {
          files.push_back(p.assign(path).append("\\").append(fileinfo.name) );
          GetAllFiles( p.assign(path).append("\\").append(fileinfo.name), files );
        }
      }
      else
      {
        files.push_back(p.assign(path).append("\\").append(fileinfo.name) );
      }

    }while(_findnext(hFile, &fileinfo)  == 0);

    _findclose(hFile);
  }
}

void
  GetAllFormatFiles( std::string path, std::vector<std::string>& files,std::string format)
{
  //�ļ����
  long   hFile   =   0;
  //�ļ���Ϣ
  struct _finddata_t fileinfo;
  std::string p;
  if((hFile = _findfirst(p.assign(path).append("\\*" + format).c_str(),&fileinfo)) !=  -1)
  {
    do
    {
      if((fileinfo.attrib &  _A_SUBDIR))
      {
        if(strcmp(fileinfo.name,".") != 0  &&  strcmp(fileinfo.name,"..") != 0)
        {
          //files.push_back(p.assign(path).append("\\").append(fileinfo.name) );
          GetAllFormatFiles( p.assign(path).append("\\").append(fileinfo.name), files,format);
        }
      }
      else
      {
        files.push_back(p.assign(path).append("\\").append(fileinfo.name) );
      }
    }while(_findnext(hFile, &fileinfo)  == 0);

    _findclose(hFile);
  }
}
*/

std::vector<std::string> file_names;
std::vector<double> cloud_timestamps;
std::string output_graph_file_name = "";
std::string output_pcd_file_name = "";
std::string output_pcd_files_prefix = "";
std::string output_markings_file_name = "";

g2o::SparseOptimizer optimizerForLoadingInput;
std::vector<g2o::VertexSE3*> vertices;
std::vector<g2o::EdgeSE3*> odometryEdges;
std::vector<g2o::EdgeSE3*> edges;
std::vector<Eigen::Matrix4d> transformations;
std::vector<Eigen::Matrix4d> _gps_poses;
std::vector<Eigen::Matrix4d> _graph_poses;
std::vector<Eigen::Matrix4d> _initial_poses;

bool _enable_visualization = true;
bool _debug = false;
bool _pause_to_debug = false;
bool _save_intial_aligned_data = false;
bool _FOR_0226 = false;

bool _initial_align = false;
bool _add_initial_alignment_to_graph = false;
bool _refine = false;
bool _add_refinement_to_graph = false;
bool _close_loop = false;
bool _add_loop_closing_to_graph = false;
bool _replay = false;

int _file_start_id = -1;
int _file_end_id = -1;
int _file_id_step = 0;

double _imu_offset_yaw = -88.5; 
int _gps_idx_offset = 0;
uint _gps_vertex_id_offset = 100000;

float _ground_removal_num_iterations = 10;
float _ground_removal_distance_threshold = 1.0;
float _ground_removal_filter_rate = 0.5;

int _normal_estimation_k = 10;

float _pass_thresh_low;
float _pass_thresh_high;

float _matching_max_correspondence_distance = 5.0;
int _matching_max_iterations = 50;
float _matching_outlier_threshold = 0.5;
float _matching_transformation_epsilon = 0.5; // 0.1 5.0
float _matching_step_size = 0.1; // 0.1 5.0
float _matching_resolution = 0.5; // 0.5 50.0

uint _refine_num_neighbor_to_consider = 5;

float _closeloop_allowed_overlap_distance = 10.0;
float _closeloop_max_correspondence_distance = 20.0;

float _translation_error_default = 0.5;
float _translation_error_factor = 1.0;
float _rotation_error_default = 1.0;
float _rotation_error_factor = 1.0;

bool _do_filter = false;
bool _remove_z_offset = false;
float param1 = 0.0;
float param2 = 0.0;
float param3 = 0.0;

bool HAVE_GPS_INPUT = false;
bool HAVE_GRAPH_INPUT = false;
bool HAVE_PENDING_GRAPH = false;

g2o::BlockSolverX::LinearSolverType * linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
g2o::BlockSolverX* blockSolver = new g2o::BlockSolverX(linearSolver);
// g2o::OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
g2o::SparseOptimizer graphOptimizer;
g2o::SparseOptimizer pendingGraphOptimizer;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Alignment Main Viewer"));

inline double
  clamp_angle(double alpha) 
{
    double _pi = 3.1415926535;
    return alpha > 0 ?
        fmod(alpha+_pi, 2*_pi) - _pi
    :
        fmod(alpha-_pi, 2*_pi) + _pi;
}

void
  SaveMarkings( const char* file, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds, const int precision = 8 )
{
  if (clouds.size()==0)
  {
    std::cout << "Output cloud is empty!" << std::endl;
    return;
  }

  std::ofstream fs;
  fs.open(file);

  if (!fs.is_open() || fs.fail())
  {
    std::cout << "Cannot open file for writing!" << std::endl;
    return;
  }

  fs.precision(precision);
  fs.imbue(std::locale::classic());

  for (int i = 0; i < clouds.size(); i++)
  {
    fs << "marking " << clouds[i]->size() << "\n";
    for (int j = 0; j < clouds[i]->size(); j++)
    {
      fs << clouds[i]->points[j].x << " " << clouds[i]->points[j].y << " " << clouds[i]->points[j].z << "\n";
    }
  }

  fs.close();
  std::cout << "Markings saved" << std::endl;
}

void
  SaveMarkings( const char* file, std::vector<pcl::PointCloud<PointTypeFull>::Ptr> &clouds, const int precision = 8 )
{
  if (clouds.size()==0)
  {
    std::cout << "Output cloud is empty!" << std::endl;
    return;
  }

  std::ofstream fs;
  fs.open(file);

  if (!fs.is_open() || fs.fail())
  {
    std::cout << "Cannot open file for writing!" << std::endl;
    return;
  }

  fs.precision(precision);
  fs.imbue(std::locale::classic());

  for (int i = 0; i < clouds.size(); i++)
  {
    fs << "marking " << clouds[i]->size() << "\n";
    for (int j = 0; j < clouds[i]->size(); j++)
    {
      fs << clouds[i]->points[j].x << " " << clouds[i]->points[j].y << " " << clouds[i]->points[j].z << "\n";
    }
  }

  fs.close();
  std::cout << "Markings saved" << std::endl;
}

void 
  GPSToUTM(double Lon, double Lat, double &easting, double &northing, 
           uint8_t &zone, bool &northenHemi, int8_t or_zone)
{
  const double a = 6378137, b = 6356752.314245; //WGS84 axes
  // const double a = 6378137.0, b = 6356752.314;
  const double e = sqrt(1 - (b/a)*(b/a) );
  const double e2 = e*e;
  const double e4 = e2*e2;
  const double e6 = e4*e2;
  const double k0 = 0.9996;
  const double FE = 500000;

  double lon = Lon*0.017453293;//degToRad(Lon);
  double lat = Lat*0.017453293;//degToRad(Lat);

  double Lon0;
  if(or_zone > 0) {
          Lon0 = 6 * or_zone - 183;
  } else {
          Lon0 = floor(Lon / 6) * 6 + 3;
  }

  double lon0 = Lon0 * 0.017453293;//degToRad(Lon0);

  double FN = (Lat < 0) * 10000000;

  double eps = e2 / (1 - e2);
  double N = a / sqrt(1 - e2 * sin(lat)*sin(lat) );
  double T = tan(lat)*tan(lat);
  double T2 = T*T;
  double C = ( (e2) / (1 - e2) ) * (cos(lat)*cos(lat) );
  double C2 = C*C;
  double A = (lon - lon0) * cos(lat);
  double A2 = A*A;
  double A3 = A2*A;
  double A4 = A2*A2;
  double A5 = A3*A2;
  double A6 = A5*A;

  double M = a * ( (1 - e2/4 - 3*e4/64 - 5*e6/256) * lat -
                             (3*e2/8 + 3*e4/32 + 45*e6/1024) * sin(2*lat) +
                             (15*e4/256 + 45*e6/1024) * sin(4*lat) -
                             (35*e6/3072) * sin(6*lat) );

  easting = FE + k0*N*(A + (1-T+C)*A3/6 + (5-18*T+T2+72*C-58*eps)*A5/120);
  northing = FN + k0*M + k0*N*tan(lat)*(A2/2 + (5-T+9*C+4*C2)*A4/24 + (61-58*T+T2+600*C-330*eps)*A6/720);
  zone = (uint8_t)(floor(Lon0/6) + 31);
  northenHemi = (Lat >= 0);
}

bool
  loadGPSPoses(const std::string& file_name)
{

  // auto tile_description_filename = std::string("markings.txt");
  using boost::filesystem::path;
  auto tempPath = path(file_name);//path(dir) / path(tile_description_filename);
  std::ifstream infile(tempPath.native(), std::ofstream::binary);

  std::cout << "Loading " << file_name.c_str() << std::endl;

  if (!infile.is_open()){
    std::cout << "cannot open GPS pose file!" << std::endl;
    return false;
  }

  int line_count = 0;
  std::string str;
  while (std::getline(infile, str))
  {
    if (!str.length()) continue;

    std::stringstream istrstr;
    istrstr.str(str.c_str());

    line_count++;
    int week, Q, NS;
    double Longitude, Latitude, HEll, VNorth, VEast, VUp, Heading, Pitch, Roll, PDOP, StdDev, HdngSD, PithchSD, RollSD;
    std::string GPSTime, iFlag, Text;
    istrstr >> week >> GPSTime >> Longitude >> Latitude >> HEll >> VNorth >> VEast >> VUp >> Heading >> Pitch >> Roll >> Q >> NS >> PDOP >> iFlag >> StdDev >> HdngSD >> PithchSD >> RollSD >> Text;

    double easting, northing;
    uint8_t zone;
    bool northenHemi;
    GPSToUTM(Longitude, Latitude, easting, northing, zone, northenHemi,-1);
    // WGS84ToGaussKrueger(Longitude, Latitude, easting, northing);
    static double offset_x = easting;
    static double offset_y = northing;
    // std::cout << "\n UTM Pose: \n <" << easting-offset_x << ", " << northing-offset_y << ">" << std::endl;

    // std::cout << GPSTime << std::endl;
    // std::cout << "   Week    GPSTime     Longitude     Latitude    H-Ell   VNorth   VEast     VUp  Heading   Pitch    Roll Q NS   PDOP iFlag  StdDev HdngSD PitchSD RollSD Text" << std::endl;
    // std::cout << line_count << "  " << week << "\t" << GPSTime << "\t" << Longitude << "\t" << Latitude << "\t" << HEll << "\t" << VNorth << "\t" << VEast << "\t" << VUp << "\t" << Heading << "\t" << Pitch << "\t" << Roll << "\t" << Q << "\t" << NS << "\t" << PDOP << "\t" << iFlag << "\t" << StdDev << "\t" << HdngSD << "\t" << PithchSD << "\t" << RollSD << "\t" << Text << std::endl;

    Eigen::AngleAxisf init_rotation (clamp_angle((90-Heading+_imu_offset_yaw)*0.017453293), Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (easting-offset_x, northing-offset_y, 0);
    Eigen::Matrix4f gps_pose_mat = (init_translation * init_rotation).matrix ();
    _gps_poses.emplace_back(gps_pose_mat.cast<double>());
    
    
    // //    Week    GPSTime     Longitude     Latitude    H-Ell   VNorth   VEast     VUp  Heading   Pitch    Roll Q NS   PDOP iFlag  StdDev HdngSD PitchSD RollSD Text 
    // // (weeks)      (sec)         (deg)        (deg)      (m)    (m/s)   (m/s)   (m/s)    (deg)   (deg)   (deg)       (dop)           (m)  (deg)   (deg)  (deg)  
    // //    1833  369699.00 116.296029997  40.05022606   44.244    0.001  -0.002  -0.000  -18.889   0.349   0.545 2 12   1.34 ZUPT    0.019  0.014   0.003  0.003 $C8$ 
    // sleep(1);
  }

  if (_gps_poses.empty())
    return false;
  else
    return true;
}

Eigen::Matrix4d
  getGPSPose(const uint pcd_file_idx)
{
  double cloud_timestamp = cloud_timestamps[pcd_file_idx];

  int pose_id = (cloud_timestamp - 2485)*20 + 39 + _gps_idx_offset;
  // std::cout << "gps pose number: " << _gps_poses.size() << std::endl;
  // std::cout << "time_stamp: " << cloud_time_stamp << " pose_id: " << pose_id << std::endl;

  // if ( pose_id < 0 )
  // {
  //   std::cout << "pose_id = " << pose_id << std::endl;
  //   pose_id=0;
  //   // std::cout << _gps_poses[(pose_id>0)?pose_id:0] << std::endl;
  // }
  return _gps_poses[(pose_id<0)?0:((pose_id<((int)(_gps_poses.size())))?pose_id:((int)(_gps_poses.size()-1)))];
  // _initial_poses.emplace_back(_gps_poses[(pose_id>0)?pose_id:0]);
}

bool
  loadPendingGraph(const std::string& file_name)
{
  // g2o::SparseOptimizer optimizerForLoadingInput;
  std::ifstream ifs(file_name);
  if (!ifs)
  {
    std::cerr << "unable to open " << file_name << ". So no pose will be used! " << std::endl;
    return false;
  }
  else
  {
    pendingGraphOptimizer.load(ifs);
    if (pendingGraphOptimizer.vertices().empty())
    {
      std::cerr << "No raw graph pose loaded. " << std::endl;
      return false;
    }
    else
    {
      std::cout << pendingGraphOptimizer.vertices().size() << " raw graph poses loaded." << std::endl;
      return true;
    }
  }
}

bool
  loadGraphPoses(const std::string& file_name)
{
  // g2o::SparseOptimizer optimizerForLoadingInput;
  std::ifstream ifs(file_name);
  if (!ifs)
  {
    std::cerr << "unable to open " << file_name << ". So no pose will be used! " << std::endl;
    return false;
  }
  else
  {
    optimizerForLoadingInput.load(ifs);
    graphOptimizer.load(ifs);
    if (optimizerForLoadingInput.vertices().empty())
    {
      std::cerr << "No graph pose loaded. " << std::endl;
      return false;
    }
    else
    {
      std::cout << optimizerForLoadingInput.vertices().size() << " graph poses loaded." << std::endl;
      return true;
    }
  }
}

Eigen::Matrix4d
  getGraphPose(g2o::SparseOptimizer& optimizer, const uint vertex_id)
{
  g2o::VertexSE3* vp = dynamic_cast<g2o::VertexSE3*>(optimizer.vertices().find(vertex_id)->second);
  Eigen::Isometry3d iso3 = vp->estimate();
  Eigen::Matrix4d t (Eigen::Matrix4d::Identity ());
  t.block<3,3>(0,0) = iso3.linear();//.cast<float>();
  t.block<3,1>(0,3) = iso3.translation();//.cast<float>();
  return t;
  // _graph_poses.emplace_back(t);
}

bool
  getLoopClosingPoseIds(g2o::SparseOptimizer& optimizer, const uint vertex_id, std::vector<uint>& pose_id_vec)
{
  std::set<g2o::HyperGraph::Edge*>& edges = (optimizer.vertices().find(vertex_id)->second)->edges();
  std::cout << edges.size() << " edges for vertex " << vertex_id << std::endl;
  for (std::set<g2o::HyperGraph::Edge*>::iterator edge_iter = edges.begin(); edge_iter != edges.end(); edge_iter++)
  {
    auto& vertices_of_cur_edge = (*edge_iter)->vertices();
    for (uint v_id = 0; v_id < vertices_of_cur_edge.size(); v_id++)
    {
      if((int)(vertex_id) - (int)(vertices_of_cur_edge[v_id]->id()) > 10)
        pose_id_vec.emplace_back(vertices_of_cur_edge[v_id]->id());
    }
  }
  return true;
}

void
  addGraphVertex(g2o::SparseOptimizer& optimizer, const uint vertex_id, const Eigen::Matrix4d pose_matrix, bool fixed=false)
{
  g2o::VertexSE3* v = new g2o::VertexSE3;
  v->setId(vertex_id);
  v->setFixed(fixed);
  // Eigen::AngleAxisd rotz(0.5, Eigen::Vector3d::UnitZ());
  // Eigen::AngleAxisd roty(-0.5, Eigen::Vector3d::UnitY());
  // Eigen::Matrix3d rot = (rotz * roty).toRotationMatrix();
  Eigen::Isometry3d iso3;
  iso3 = pose_matrix.block<3,3>(0,0);//.cast<double>();
  iso3.translation() = pose_matrix.block<3,1>(0,3);//.cast<double>();
  // iso3 = rot;
  // iso3.translation() = iso3.linear() * Eigen::Vector3d(3, 0, 0);
  v->setEstimate(iso3);
  vertices.push_back(v);
  std::cout << "added: [vertex " << vertex_id << "] Fixed? " << v->fixed() << std::endl;;
  v->write(std::cout);
  std::cout << std::endl;

  optimizer.addVertex(v);
}

void
  addGraphEdge( g2o::SparseOptimizer& optimizer,
                const uint pre_vertex_idx,
                const uint cur_vertex_idx,
                const Eigen::Matrix4d relative_pose_matrix,
                const std::vector<float> std_var_vec )
{
  Eigen::Matrix3d transNoise = Eigen::Matrix3d::Zero();
  transNoise(0, 0) = std::pow(std_var_vec[0], 2);
  transNoise(1, 1) = std::pow(std_var_vec[1], 2);
  transNoise(2, 2) = std::pow(std_var_vec[2], 2);
  // std::cout << "translation noise: " << std::pow(transNoise(0, 0),0.5) << std::endl;

  Eigen::Matrix3d rotNoise = Eigen::Matrix3d::Zero();
  rotNoise(0, 0) = std::pow(std_var_vec[3], 2);
  rotNoise(1, 1) = std::pow(std_var_vec[4], 2);
  rotNoise(2, 2) = std::pow(std_var_vec[5], 2);
  // std::cout << "rotation noise: " << std::pow(rotNoise(0, 0),0.5) << std::endl;

  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  information.block<3,3>(0,0) = transNoise.inverse();
  information.block<3,3>(3,3) = rotNoise.inverse();

  g2o::VertexSE3* prev = dynamic_cast<g2o::VertexSE3*>(optimizer.vertices().find(pre_vertex_idx)->second);
  g2o::VertexSE3* cur = dynamic_cast<g2o::VertexSE3*>(optimizer.vertices().find(cur_vertex_idx)->second);
//  g2o::VertexSE3* prev = vertices[pre_vertex_idx];
//  g2o::VertexSE3* cur  = vertices[cur_vertex_idx];
  // Eigen::Isometry3d iso3_edge = prev->estimate().inverse() * cur->estimate();

  Eigen::Isometry3d iso3_edge;
  iso3_edge = relative_pose_matrix.block<3,3>(0,0);//.cast<double>();
  iso3_edge.translation() = relative_pose_matrix.block<3,1>(0,3);//.cast<double>();

  g2o::EdgeSE3* e = new g2o::EdgeSE3;
  e->setVertex(0, prev);
  e->setVertex(1, cur);
  e->setMeasurement(iso3_edge);
  e->setInformation(information);

  // g2o::VertexSE3* from = static_cast<g2o::VertexSE3*>(e->vertex(0));
  // g2o::VertexSE3* to = static_cast<g2o::VertexSE3*>(e->vertex(1));
  // g2o::HyperGraph::VertexSet aux; aux.insert(from);
  // e->initialEstimate(aux, to);
  odometryEdges.push_back(e);
  edges.push_back(e);

  std::cout << "added [edge " << prev->id() << "-" << cur->id() << "] ";
  e->write(std::cout);
  std::cout << endl;

  optimizer.addEdge(e);
}

void
  GetAllFormatFiles( std::string path, std::vector<std::string>& files, std::vector<double>& timestamps, std::string extension)
{
  struct dirent *ptr;
  DIR *dir;
  dir=opendir(path.c_str());
  while((ptr=readdir(dir))!=NULL)
  {
    //跳过'.'和'..'两个目录
    std::string f_name = ptr->d_name;
    if(ptr->d_name[0] == '.'||f_name.compare (f_name.size () - extension.size (), extension.size (), extension) != 0)
        continue;
    //cout << ptr->d_name << endl;

    if (_FOR_0226)
    {
    /////////////////////////////////////////////////////////////////////////////////
    //////////////////////// temp code for 0226 dataset /////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////
      uint name_id_pre = std::atoi(ptr->d_name);
      // std::cout << "before: " << name_id_pre;
      uint64_t name_id = name_id_pre;
      // std::cout << " before2: " << name_id;
      if (name_id < 2000000000) name_id = name_id_pre + 3600000000;
      // std::cout << "  intermidiate:" << name_id;
      char temp[10];
      // sprintf(temp, "%d", name_id);
      sprintf(temp, "%" PRIu64, name_id);
      std::string str(temp);
      std::cout << "  after:" << str << std::endl << std::endl;
      // sleep(1);
      files.push_back(str);
    /////////////////////////////////////////////////////////////////////////////////
    ////////////////////// end temp code for 0226 dataset ///////////////////////////
    /////////////////////////////////////////////////////////////////////////////////
    }
    else
    {
      files.push_back(path+'/'+ptr->d_name);
    }
  }

  sort(files.begin(), files.end());

  if (_FOR_0226)
  {
    for (uint i = 0; i < files.size(); ++i)
    {
      /////////////////////////////////////////////////////////////////////////////////
      //////////////////////// temp code for 0226 dataset /////////////////////////////
      /////////////////////////////////////////////////////////////////////////////////
      std::string ttname = files[i];
      // std::cout << " 1:" << ttname;
      unsigned long long name_id = std::atoll(ttname.c_str());
      double cloud_timestamp = ((double)name_id)/1000000.0;
      // std::cout << " 2:" << name_id;
      if (name_id > 3600000000) name_id = name_id - 3600000000;
      // std::cout << " 3:" << name_id;
      uint name_id_af = name_id;
      // std::cout << " 4:" << name_id_af;
      char temp[10];
      sprintf(temp, "%u", name_id_af);
      // sleep(1);
      // sprintf(temp, "%" PRIu64, name_id);
      files[i] = path + "/" + temp + ".pcd";
      // std::cout << files[i] << std::endl;
      // pcl::io::loadPCDFile (files[i], *temp_cloud_ptr);
      timestamps.emplace_back(cloud_timestamp);
      /////////////////////////////////////////////////////////////////////////////////
      ////////////////////// end temp code for 0226 dataset ///////////////////////////
      /////////////////////////////////////////////////////////////////////////////////

      // cout << files[i] << endl;
    }
    std::cout << std::endl;
    assert(files.size()==timestamps.size());
  }
  closedir(dir);
}

void
  showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*                         3D Mapping - Usage Guide                        *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                          Show this help." << std::endl;
}

void
  parseCommandLine (int argc, char *argv[])
{
  // Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }
  /////////////////////////////////////////////
  ////////// Set program behaviors ////////////
  /////////////////////////////////////////////
  if (pcl::console::find_switch (argc, argv, "-v"))
  {
    _enable_visualization = true;
  }

  if (pcl::console::find_switch (argc, argv, "-debug"))
  {
    _debug = true;
  }

  if (pcl::console::find_switch (argc, argv, "-p_debug"))
  {
    _pause_to_debug = true;
  }

  if (pcl::console::find_switch (argc, argv, "-o_init"))
  {
    _save_intial_aligned_data = true;
  }

  if (pcl::console::find_switch (argc, argv, "-0226"))
  {
    _FOR_0226 = true;
  }

  //// initial alignment
  if (pcl::console::find_switch (argc, argv, "-i"))
  {
    _initial_align = true;
  }
  if (pcl::console::find_switch (argc, argv, "-ig"))
  {
    _initial_align = true;
    _add_initial_alignment_to_graph = true;
  }

  //// refinement
  if (pcl::console::find_switch (argc, argv, "-r"))
  {
    _refine = true;
  }
  if (pcl::console::find_switch (argc, argv, "-rg"))
  {
    _refine = true;
    _add_refinement_to_graph = true;
  }

  //// loop closing
  if (pcl::console::find_switch (argc, argv, "-c"))
  {
    _close_loop = true;
  }
  if (pcl::console::find_switch (argc, argv, "-cg"))
  {
    _close_loop = true;
    _add_loop_closing_to_graph = true;
  }

  //// replay
  if (pcl::console::find_switch (argc, argv, "-replay"))
  {
    _replay = true;
  }

  pcl::console::parse_argument (argc, argv, "--output_graph", output_graph_file_name);
  pcl::console::parse_argument (argc, argv, "--output_pcd", output_pcd_file_name);
  pcl::console::parse_argument (argc, argv, "--output_pcds_prefix", output_pcd_files_prefix);
  pcl::console::parse_argument (argc, argv, "--output_markings", output_markings_file_name);

  /////////////////////////////////////////////
  ///////// Set algorithm parameters //////////
  /////////////////////////////////////////////

  //// gps data reading
  pcl::console::parse_argument (argc, argv, "--imu_offset_yaw", _imu_offset_yaw);
  pcl::console::parse_argument (argc, argv, "--idx_offset", _gps_idx_offset);
  
  //// ground_removal
  pcl::console::parse_argument (argc, argv, "--ground_removal_num_iterations", _ground_removal_num_iterations);
  pcl::console::parse_argument (argc, argv, "--ground_removal_distance_threshold", _ground_removal_distance_threshold);
  pcl::console::parse_argument (argc, argv, "--ground_removal_filter_rate", _ground_removal_filter_rate);

  //// normal estimation
  pcl::console::parse_argument (argc, argv, "--normal_estimation_k", _normal_estimation_k);

  //// pass_filter
  pcl::console::parse_argument (argc, argv, "--pass_thresh_low", _pass_thresh_low);
  pcl::console::parse_argument (argc, argv, "--pass_thresh_high", _pass_thresh_high);

  //// scan-matching
  pcl::console::parse_argument (argc, argv, "--matching_max_correspondence_distance", _matching_max_correspondence_distance);
  pcl::console::parse_argument (argc, argv, "--matching_max_iterations", _matching_max_iterations);
  pcl::console::parse_argument (argc, argv, "--matching_outlier_threshold", _matching_outlier_threshold);
  pcl::console::parse_argument (argc, argv, "--matching_transformation_epsilon", _matching_transformation_epsilon);
  pcl::console::parse_argument (argc, argv, "--matching_step_size", _matching_step_size);
  pcl::console::parse_argument (argc, argv, "--matching_resolution", _matching_resolution);

  //// refinement
  pcl::console::parse_argument (argc, argv, "--refine_num_neighbor_to_consider", _refine_num_neighbor_to_consider);

  //// loop closing
  pcl::console::parse_argument (argc, argv, "--closeloop_allowed_overlap_distance", _closeloop_allowed_overlap_distance);
  pcl::console::parse_argument (argc, argv, "--closeloop_max_correspondence_distance", _closeloop_max_correspondence_distance);

  pcl::console::parse_argument (argc, argv, "--translation_error_default", _translation_error_default);
  pcl::console::parse_argument (argc, argv, "--translation_error_factor", _translation_error_factor);
  pcl::console::parse_argument (argc, argv, "--rotation_error_default", _rotation_error_default);
  pcl::console::parse_argument (argc, argv, "--rotation_error_factor", _rotation_error_factor);
  //// ground extraction
  // pcl::console::parse_argument (argc, argv, "--ground_extraction_non_ground_grid_size", _ground_extraction_non_ground_grid_size);
  // pcl::console::parse_argument (argc, argv, "--ground_extraction_ground_grid_size", _ground_extraction_ground_grid_size);
  // pcl::console::parse_argument (argc, argv, "--ground_extraction_max_iterations", _ground_extraction_max_iterations);
  // pcl::console::parse_argument (argc, argv, "--ground_extraction_distance_threshold", _ground_extraction_distance_threshold);
  
  //// segmentation
  // pcl::console::parse_argument (argc, argv, "--segmentation_cluster_min_size", _segmentation_cluster_min_size);

  if (pcl::console::find_switch (argc, argv, "-do_filter"))
  {
    _do_filter = true;
  }
  if (pcl::console::find_switch (argc, argv, "-remove_z_offset"))
  {
    _remove_z_offset = true;
  }
  pcl::console::parse_argument (argc, argv, "--param1", param1);
  pcl::console::parse_argument (argc, argv, "--param2", param2);
  pcl::console::parse_argument (argc, argv, "--param3", param3);

  /////////////////////////////////////////////
  //////////////// Read files /////////////////
  /////////////////////////////////////////////

  /////////////////////////
  //////// gps file ///////
  /////////////////////////
  std::string gps_file_name;
  pcl::console::parse_argument (argc, argv, "--gps", gps_file_name);
  if ( !gps_file_name.empty() )
  {
    HAVE_GPS_INPUT = loadGPSPoses(gps_file_name);
  }

  /////////////////////////
  /////  graph file  //////
  /////////////////////////
  std::string graph_file_name;
  pcl::console::parse_argument (argc, argv, "--init_graph", graph_file_name);
  if ( !graph_file_name.empty() )
  {
    HAVE_GRAPH_INPUT = loadGraphPoses(graph_file_name);
  }

  std::string pending_graph_file_name;
  pcl::console::parse_argument (argc, argv, "--pending_graph", pending_graph_file_name);
  if ( !pending_graph_file_name.empty() )
  {
    if ( pending_graph_file_name.compare ("NEW") )
      HAVE_PENDING_GRAPH = loadPendingGraph(pending_graph_file_name);
  }
  else {
    if (HAVE_GRAPH_INPUT)
      HAVE_PENDING_GRAPH = loadPendingGraph(graph_file_name);
  }

  /////////////////////////
  /////// pcd files ///////
  /////////////////////////

    // Parse directories
  std::vector<std::string> directory_names;
  pcl::console::parse_multiple_arguments(argc, argv, "--pcd_dir", directory_names);
  std::cout << directory_names.size() << " directories parsed." << std::endl;
  for (uint i = 0; i < directory_names.size(); i++)
  {
    std::vector<std::string> temp_file_names;
    std::vector<double> temp_timestamps;

    GetAllFormatFiles(directory_names[i], temp_file_names, temp_timestamps, "");

    file_names.insert(file_names.end(), temp_file_names.begin(), temp_file_names.end());
    cloud_timestamps.insert(cloud_timestamps.end(), temp_timestamps.begin(), temp_timestamps.end());
  }
    // Parse individual file names
  // std::vector<int> file_name_indices;
  // file_name_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  // for (uint i = 0; i < file_name_indices.size (); i++)
  // {
  //   file_names.emplace_back(argv[file_name_indices[i]]);
  // }
  // std::cout << file_names.size() << " files to be loaded." << std::endl;

  /////////////////////////////////////////////
  ////// Select data indices to be played /////
  /////////////////////////////////////////////
  pcl::console::parse_argument (argc, argv, "--start", _file_start_id);
  pcl::console::parse_argument (argc, argv, "--end", _file_end_id);
  pcl::console::parse_argument (argc, argv, "--step", _file_id_step);
  
  int _id1=-1, _id2=-1;
  pcl::console::parse_argument (argc, argv, "--id1", _id1);
  pcl::console::parse_argument (argc, argv, "--id2", _id2);
  if (_id1!=-1&&_id2!=-1)
  {
    _file_start_id = _id1;
    _file_end_id = _id2;
    _file_id_step = _id2-_id1;
  }

  if (_file_start_id < 0) _file_start_id = 0;
  if (_file_end_id < 0) _file_end_id = file_names.size()-1;
  if (_file_id_step == 0) _file_id_step = 1;
}

bool
  loadPCDFile(const std::string file_name, pcl::PointCloud<PointTypeIO>::Ptr &cloud_ptr)
{
  std::string extension (".pcd");
  // std::string fname = files[_file_start_id], ofname;
  if (file_name.size () <= extension.size ())
  {
    std::cerr << "file name too short: " << file_name << std::endl;
    return false;
  }
  //std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
  if (file_name.compare (file_name.size () - extension.size (), extension.size (), extension) == 0)
  {
    // Load the cloud and saves it into the global list of models
    pcl::io::loadPCDFile (file_name, *cloud_ptr);
    // ofname = fname;
    // ofname.insert(ofname.size()-4,"_ndt");
    if (cloud_ptr->empty())
    {
      std::cerr << "cloud is empty: " << file_name << std::endl;
      return false;
    }
    else
    {
      return true;
    }
  }
  else
    return false;
}

void
  loadData (int argc, char **argv, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds)
{
  std::string extension (".pcd");
  // Suppose the first argument is the actual test model
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::io::loadPCDFile (argv[i], *cloud);

      clouds.push_back (cloud);
    }
  }
}

void
  loadData (std::vector<std::string> files, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds)
{
  std::string extension (".pcd");
  // Suppose the first argument is the actual test model
  for (uint i = 0; i < files.size(); i++)
  {
    std::string fname = files[i];
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::io::loadPCDFile (fname, *cloud);

      clouds.push_back (cloud);
    }
  }
}

void
  WriteGraphFile( std::string outFilename )
{
  // write output
  std::ofstream fileOutputStream;
  if (outFilename != "-") {
    std::cout << "Writing into " << outFilename << std::endl;
    fileOutputStream.open(outFilename.c_str());
  } else {
    std::cout << "writing to stdout" << std::endl;
  }

  std::string vertexTag = g2o::Factory::instance()->tag(vertices[0]);
  std::string edgeTag = g2o::Factory::instance()->tag(edges[0]);

  std::ostream& fout = outFilename != "-" ? fileOutputStream : cout;
  for (size_t i = 0; i < vertices.size(); ++i) {
    g2o::VertexSE3* v = vertices[i];
    fout << vertexTag << " " << v->id() << " ";
    v->write(fout);
    fout << std::endl;
    if (v->fixed()) {
      fout << "FIX " << v->id() << std::endl;
    }
  }

  for (size_t i = 0; i < edges.size(); ++i) {
    g2o::EdgeSE3* e = edges[i];
    g2o::VertexSE3* from = static_cast<g2o::VertexSE3*>(e->vertex(0));
    g2o::VertexSE3* to = static_cast<g2o::VertexSE3*>(e->vertex(1));
    fout << edgeTag << " " << from->id() << " " << to->id() << " ";
    e->write(fout);
    fout << std::endl;
  }
}

void
  normalize_intensity (pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
  for (unsigned int i = 0; i<cloud->size(); i++){
    if (cloud->points[i].intensity>2)
    {
      cloud->points[i].intensity = 2.111;
      std::cout<< "+";
    }
    if (cloud->points[i].intensity<-2)
    {
      cloud->points[i].intensity = -2.111;
      std::cout<< "-";
    }
  }
}

void
  remove_ground (const pcl::PointCloud<PointTypeIO>::Ptr &cloud_in_ptr,
                 pcl::PointCloud<PointTypeIO>::Ptr &cloud,
                 pcl::PointCloud<PointTypeIO>::Ptr &ground)
{

  //pcl::PointCloud<PointTypeIO>::Ptr cloud (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeIO>::Ptr cloud_v (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeIO>::Ptr cloud_p (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeIO>::Ptr cloud_f (new pcl::PointCloud<PointTypeIO>);

  pcl::copyPointCloud(*cloud_in_ptr, *cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<PointTypeIO> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (_ground_removal_num_iterations);
  seg.setDistanceThreshold (_ground_removal_distance_threshold);

  // Create the filtering object
  pcl::ExtractIndices<PointTypeIO> extract;

  int i = 0, nr_points = (int) cloud_in_ptr->points.size ();
  // While 30% of the original cloud is still there
  while (cloud->points.size () > _ground_removal_filter_rate * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    // std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    *ground = *ground + *cloud_p;

    for (uint j = 0; j<cloud_p->size(); j++)
      cloud_p->points[j].intensity = 0;

    //std::stringstream ss;
    //ss << "table_scene_lms400_plane_" << i << ".pcd";
    //writer.write<pcl::PointXYZI> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud.swap (cloud_f);
    i++;

    //*cloud_v = *cloud + *cloud_p;

    //viewer->setCameraPosition(centroid[0]-40.0, centroid[1], centroid[2]+20.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(cloud_v, "intensity");
    //viewer->updatePointCloud<pcl::PointXYZI>(cloud_v, intensity_distribution, "cloud");
    //viewer->removeAllPointClouds();
    //viewer->addPointCloud(wholeCloud);

  }

  pcl::VoxelGrid<pcl::PointXYZI> gridfilter;
  gridfilter.setInputCloud (ground);
  gridfilter.setLeafSize (1.0f, 1.0f, 1.0f);
  gridfilter.filter (*ground);

  //pcl::VoxelGrid<pcl::PointXYZI> gridfilter;
  gridfilter.setInputCloud (cloud);
  gridfilter.setLeafSize (0.3f, 0.3f, 0.3f);
  gridfilter.filter (*cloud);

   pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
   sor.setInputCloud (cloud);
   sor.setMeanK (3);
   sor.setStddevMulThresh (0.0);
   sor.setNegative(false);
   sor.filter (*cloud);
  // sor.setNegative(true);
  // sor.filter (*removedPoints);

   pcl::PassThrough<pcl::PointXYZI> pass;
   pass.setInputCloud (cloud);
   pass.setFilterFieldName ("x");
   pass.setFilterLimits (-2.0, 2.0);
   pass.setFilterFieldName ("y");
   pass.setFilterLimits (-2.0, 2.0);
   pass.setFilterLimitsNegative (true);
   pass.filter (*cloud);
}

bool
  QSRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  if (squared_distance<param3 && ((int)(point_a.data[0]/20))==((int)(point_b.data[0]/20)) /*&& point_a.y/20==point_b.y/20*/ ) return true;
//  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal, v_a = point_a.data, v_b = point_b.data;
//
//  float d_ab = sqrt(squared_distance);
//
//  float curvature = 2 * sin( 0.5 * acos(point_a_normal.dot (point_b_normal)) ) / d_ab;
//  float cos_na_ab = point_a_normal.dot( v_a - v_b ) / d_ab;
//  float cos_nb_ba = point_b_normal.dot( v_b - v_a ) / d_ab;
//
//  //std::cerr << "curvature: " << curvature << "\n" ;
//  //std::cerr << "d_a: " << d_a << " d_b:" << d_b << "\n" ;
//
//  if ( curvature < 1.0/1.0/*outdoor:1.0/3.0*/ && fabs(cos_na_ab) < 0.15/*outdoor:0.15*/ && fabs(cos_nb_ba) < 0.15/*outdoor:0.15*/ /*&& fabs (point_a.intensity - point_b.intensity) < 6.0f*/ )
//  {
//    //std::cerr << "==========================================curvature: " << curvature << "\n" ;
//    return (true);
//  }
  //  if (squared_distance < 10000)
  //  {
  //    //std::cerr << "_" ;
  //    if (fabs (point_a.intensity - point_b.intensity) < 8.0f)
  //    {
  //      std::cerr << "111           curvature: " << curvature << "\n" ;
  //      return (true);
  //    }
  // //     if (fabs (point_a_normal.dot (point_b_normal)) > 0.995)
  // //       return (true);
  //  }
  //  else
  //  {
  //    //std::cerr << "Angular Distance: " << point_a_normal.dot (point_a_normal);
  //    //std::cerr << "+" ;
  //    if (fabs (point_a.intensity - point_b.intensity) < 3.0f)
  //    {
  //      std::cerr << "222222        curvature: " << curvature << "\n" ;
  //      return (true);
  //    }
  // //     if (fabs (point_a_normal.dot (point_b_normal)) > 0.98)
  // //       return (true);
  //  }
  //  std::cerr << "333333333     curvature: " << curvature << "\n" ;
  return (false);
}

void
  segment_cloud (const pcl::PointCloud<PointTypeIO>::Ptr &cloud_in_ptr,
          std::vector<pcl::PointCloud<PointTypeFull>::Ptr> &segmented_component_ptrs,
          pcl::PointCloud<PointTypeFull>::Ptr &useful_cloud_ptr,
          pcl::PointCloud<PointTypeFull>::Ptr &cloud_for_show)
{
  //pcl::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl::PointCloud<PointTypeIO>), cloud_out (new pcl::PointCloud<PointTypeIO>), cloud_downsampled (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
  pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);

  pcl::copyPointCloud(*cloud_in_ptr, *cloud_with_normals);
  pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
  ne.setInputCloud (cloud_in_ptr);
  ne.setSearchMethod (search_tree);
  //ne.setRadiusSearch (300);
  ne.setKSearch(10); //outdoor:10
  ne.compute (*cloud_with_normals);

  // std::cout << ">> normal calculation ... done" << std::endl;

  pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
  cec.setInputCloud (cloud_with_normals);
  cec.setConditionFunction (&QSRegionGrowing);
  cec.setClusterTolerance (1.0);
  cec.setMinClusterSize (10); //outdoor:cloud_with_normals->points.size () / 2000
  cec.setMaxClusterSize (cloud_with_normals->points.size () / 5);
  cec.segment (*clusters);
  cec.getRemovedClusters (small_clusters, large_clusters);

  // std::cout << ">> segmentation ... done" << std::endl;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZI>::Ptr mls_cloudXYZI (new pcl::PointCloud<pcl::PointXYZI>);
  //pcl::PointCloud<pcl::PointXYZI>::Ptr mls_cloudXYZI_upsampled (new pcl::PointCloud<pcl::PointXYZI>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr previous_output (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*cloud_with_normals, *cloud_for_show);

  // std::cout << ">> copy cloud ... done" << std::endl;
  // std::cout << ">> number of small clusters: " << small_clusters->size () << std::endl;
  // std::cout << ">> number of large clusters: " << large_clusters->size () << std::endl;
  // std::cout << ">> number of target clusters: " << clusters->size () << std::endl;

  // Using the intensity channel for lazy visualization of the output
  for (uint i = 0; i < small_clusters->size (); ++i){

    pcl::PointCloud<PointTypeFull>::Ptr current_cloud_i (new pcl::PointCloud<PointTypeFull>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (uint j = 0; j < (*small_clusters)[i].indices.size (); ++j){
      cloud_for_show->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
      //std::cout << ">> emplace_back now ... " << cloud_for_show->points[(*small_clusters)[i].indices[j]].x << " " << cloud_for_show->points[(*small_clusters)[i].indices[j]].y << " " << cloud_for_show->points[(*small_clusters)[i].indices[j]].z << std::endl;
      current_cloud_i->points.emplace_back(cloud_with_normals->points[(*small_clusters)[i].indices[j]]);
    }
    //pcl::copyPointCloud(*current_cloud_i, *current_cloud);
    //*mls_cloud += *current_cloud;
    //*mls_cloudXYZI += *current_cloud_i;
  }
  //pcl::copyPointCloud(*cloud_out, *previous_output);

  // std::cout << ">> step 1 .. done" << std::endl;

  for (uint i = 0; i < large_clusters->size (); ++i){

    pcl::PointCloud<PointTypeFull>::Ptr current_cloud_i (new pcl::PointCloud<PointTypeFull>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr current_mls_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (uint j = 0; j < (*large_clusters)[i].indices.size (); ++j){
      cloud_for_show->points[(*large_clusters)[i].indices[j]].intensity = 10.0;
      current_cloud_i->points.emplace_back(cloud_for_show->points[(*large_clusters)[i].indices[j]]);
    }
    segmented_component_ptrs.emplace_back(current_cloud_i);

    //pcl::copyPointCloud(*current_cloud_i,*current_cloud);
    //*mls_cloudXYZI += *current_cloud_i;

    //pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    ////mls.setComputeNormals (true);

    //std::cout << ">> step 2 .. done" << std::endl;

    //// Set parameters
    //mls.setInputCloud (current_cloud);
    //mls.setPolynomialFit (true);
    //mls.setSearchMethod (tree);
    //mls.setSearchRadius (0.2);
    //mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    //mls.setUpsamplingRadius(0.2);
    //mls.setUpsamplingStepSize(0.01);

    //std::cout << ">> step 3 .. done" << std::endl;

    //// Reconstruct
    //mls.process (*current_mls_cloud);

    //// upsample intensity
    //pcl::PointCloud<pcl::PointXYZI>::Ptr temp_XYZI (new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::copyPointCloud(*current_mls_cloud,*temp_XYZI);
    //for (unsigned int ii = 0; ii<temp_XYZI->size(); ii++ )
    //{
    //  temp_XYZI->points[ii].intensity = 10.0;
    //}
    //*mls_cloudXYZI_upsampled += *temp_XYZI;
    //// end upsample intensity

    //*mls_cloud += *current_mls_cloud;

  }
  for (uint i = 0; i < clusters->size (); ++i)
  {
    //std::cerr << "current cluster id: " << i << " of " << clusters->size () << "\n";

    pcl::PointCloud<PointTypeFull>::Ptr current_cloud_i (new pcl::PointCloud<PointTypeFull>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr current_mls_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr current_grid_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    int label = rand () % 8;
    for (uint j = 0; j < (*clusters)[i].indices.size (); ++j){
      cloud_for_show->points[(*clusters)[i].indices[j]].intensity = label;
      current_cloud_i->points.emplace_back(cloud_for_show->points[(*clusters)[i].indices[j]]);
    }

    segmented_component_ptrs.emplace_back(current_cloud_i);

    *useful_cloud_ptr = *useful_cloud_ptr + *current_cloud_i;
    ////std::cout << ">> 1 .. ";
    ////pcl::copyPointCloud(*current_cloud_i,*current_cloud);
    ////std::cout << ">> 2 .. ";
    ////*mls_cloudXYZI += *current_cloud_i;
    ////std::cout << ">> 3 .. ";
    // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    // //std::cout << ">> 4 .. ";
    // //mls.setComputeNormals (true);

    // // Set parameters
    // mls.setInputCloud (current_cloud);
    // mls.setPolynomialFit (true);
    // mls.setPolynomialOrder(1);
    // mls.setSearchMethod (tree);
    // mls.setSearchRadius (1.0);
    // mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    // mls.setUpsamplingRadius(0.5);
    // mls.setUpsamplingStepSize(0.2);
    //    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::RANDOM_UNIFORM_DENSITY);
    //    mls.setPointDensity(40);
    //std::cout << ">> 5 .. ";
    //// Reconstruct
    //mls.process (*current_mls_cloud);
    ////std::cout << ">> 6 .. ";
    //pcl::VoxelGrid<pcl::PointXYZ> c_vg;
    //c_vg.setInputCloud (current_mls_cloud);
    //c_vg.setLeafSize (0.01, 0.01, 0.01);
    ////vg.setLeafSize (0.1, 0.1, 0.1);
    //c_vg.setDownsampleAllData (true);
    //c_vg.filter (*current_grid_cloud);

    //std::cout << ">> 7 .. ";
    //// upsample intensity
    //pcl::PointCloud<pcl::PointXYZI>::Ptr temp_XYZI (new pcl::PointCloud<pcl::PointXYZI>);
    //std::cout << ">> 8 .. ";
    //pcl::copyPointCloud(*current_grid_cloud,*temp_XYZI);
    //std::cout << ">> 9 .. ";
    //for (unsigned int ii = 0; ii<temp_XYZI->size(); ii++ )
    //{
    //  temp_XYZI->points[ii].intensity = label;
    //}
    //std::cout << ">> 10 .. ";
    //*mls_cloudXYZI_upsampled += *temp_XYZI;
    //// end upsample intensity
    //std::cout << ">> 11 .. " << std::endl;
    //*mls_cloud += *current_grid_cloud;
  }

  // std::cout << "number of cloud points to show: " << cloud_for_show->size() << std::endl;
  // std::cout << "number of useful cloud points: " << useful_cloud_ptr->size() << std::endl;
}

void
  initial_align (std::vector<std::string>& files)
{
  pcl::PointCloud<PointTypeIO>::Ptr pre_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
  loadPCDFile(files[_file_start_id], pre_cloud_ptr);

  pcl::PointCloud<PointTypeIO>::Ptr pre_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeIO>::Ptr pre_ground_ptr (new pcl::PointCloud<PointTypeIO>);
  remove_ground(pre_cloud_ptr, pre_no_ground_ptr, pre_ground_ptr);

  std::vector<pcl::PointCloud<PointTypeFull>::Ptr> segmented_component_ptrs;
  pcl::PointCloud<PointTypeFull>::Ptr pre_useful_cloud_ptr (new pcl::PointCloud<PointTypeFull>);
  pcl::PointCloud<PointTypeFull>::Ptr segmented_cloud_to_show_ptr (new pcl::PointCloud<PointTypeFull>);
  segment_cloud(pre_no_ground_ptr, segmented_component_ptrs, pre_useful_cloud_ptr, segmented_cloud_to_show_ptr);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Initial Allignment Viewer"));
  if (_enable_visualization)
  {
    viewer->setBackgroundColor (0, 0, 0);
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
    //viewer->addPointCloud<pcl::PointXYZI> (whole_cloud_out, intensity_distribution, "whole cloud");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");
    //viewer->addCoordinateSystem (1.0);
    //viewer->initCameraParameters ();

    pcl::PointCloud<pcl::PointXYZ>::Ptr null_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      target_color (null_cloud_ptr, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      output_color (null_cloud_ptr, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, output_color, "output cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "output cloud");

    // Coloring and visualizing initial transformed input cloud (gray).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      temp_color (null_cloud_ptr, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, temp_color, "temp cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "temp cloud");
  }

  // Eigen::Matrix3f intrinsic (Eigen::Matrix3f::Identity ());
  // Eigen::Matrix4f extrinsic (Eigen::Matrix4f::Identity ());
  // Eigen::Matrix4d t (Eigen::Matrix4d::Identity ());
  // Eigen::Matrix4d t_total = (_initial_poses.empty()) ? Eigen::Matrix4d::Identity() : _initial_poses[_file_start_id];

  Eigen::Matrix4d t_total (Eigen::Matrix4d::Identity());
  // std::cout << "condition: " << _initial_poses.size() << " v.s. " << (1 + std::floor((idx-_file_start_id)/_file_id_step)) << std::endl;
  if (HAVE_GRAPH_INPUT)
  {
    uint query_vertex_id = _file_start_id;
    t_total = getGraphPose(optimizerForLoadingInput, query_vertex_id);
  }
  else if (HAVE_GPS_INPUT)
  {
    uint query_pcd_file_idx = _file_start_id;
    t_total = getGPSPose(query_pcd_file_idx);
  }

  // if ( _initial_poses.size() == file_names.size() )
  // {
  //   t_total = (_initial_poses[idx-_file_id_step].inverse() * _initial_poses[idx] ).cast<float>();
  // }
  // else if ( _initial_poses.size() == (1 + std::floor((_file_end_id-_file_start_id)/_file_id_step)) )
  // {
  //   t_total = (_initial_poses[std::floor((idx-_file_id_step-_file_start_id)/_file_id_step)].inverse() * _initial_poses[std::floor((idx-_file_start_id)/_file_id_step)] ).cast<float>();
  // }

  addGraphVertex(graphOptimizer, _file_start_id, t_total);
  if (!HAVE_PENDING_GRAPH)
    addGraphVertex(pendingGraphOptimizer, _file_start_id, t_total);

  uint pre_vex_id = _file_start_id;
  //Eigen::Matrix<double,4,1> centroid;
  //pcl::compute3DCentroid(*pre_Cloud, centroid);
  //Eigen::AngleAxisf init_rotation (0.5, Eigen::Vector3f::UnitY ());
  //Eigen::Translation3f init_translation (centroid[0]-2.0, centroid[1], centroid[2]+2.0);
  //extrinsic = (init_translation * init_rotation).matrix ();

  //while (!viewer->wasStopped ())
  //{
  //  viewer->spinOnce (100);
  //  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  //}

  // int tmp_counter = 0;
  for ( uint idx = (_file_id_step > 0) ? (_file_start_id+_file_id_step) : (_file_end_id-_file_id_step);
        ( _file_id_step > 0 && idx <= _file_end_id ) || ( _file_id_step < 0 && idx >= _file_start_id );
        idx = idx + _file_id_step )
  {
    pcl::PointCloud<PointTypeIO>::Ptr tmp_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
    loadPCDFile(files[idx], tmp_cloud_ptr);

    // std::cout << "debug 2 " <<tmp_cloud_ptr->size() << std::endl;
    pcl::PointCloud<PointTypeIO>::Ptr tmp_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<PointTypeIO>::Ptr tmp_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    remove_ground(tmp_cloud_ptr, tmp_no_ground_ptr, tmp_ground_ptr);

    //std::vector<pcl::PointCloud<PointTypeFull>::Ptr> segmented_component_ptrs;
    pcl::PointCloud<PointTypeFull>::Ptr tmp_useful_cloud_ptr (new pcl::PointCloud<PointTypeFull>);
    //pcl::PointCloud<PointTypeFull>::Ptr segmented_cloud_to_show_ptr (new pcl::PointCloud<PointTypeFull>);
    segment_cloud(tmp_no_ground_ptr, segmented_component_ptrs, tmp_useful_cloud_ptr, segmented_cloud_to_show_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::copyPointCloud(*tmp_no_ground_ptr/*tmp_useful_cloud_ptr*/,*input_cloud_ptr);
    // pcl::copyPointCloud(*pre_no_ground_ptr/*pre_useful_cloud_ptr*/,*target_cloud_ptr);

    pcl::copyPointCloud(*tmp_no_ground_ptr/*+*tmp_ground_ptr*/, *input_cloud_ptr);
    pcl::copyPointCloud(*pre_no_ground_ptr/*+*pre_ground_ptr*/, *target_cloud_ptr);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    // pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> matcher;
    // boost::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> > warp_fcn
    //   (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);
    // boost::shared_ptr<pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> > te
    //   (new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>);
    // te->setWarpFunction (warp_fcn);
    // matcher.setTransformationEstimation (te);
    // matcher.setRANSACOutlierRejectionThreshold (_matching_outlier_threshold);
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> matcher;
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> matcher;
    // matcher.setTransformationEpsilon (_matching_transformation_epsilon); // 0.1 5.0
    // matcher.setStepSize (_matching_step_size); // 0.1 5.0
    // matcher.setResolution (_matching_resolution); // 0.5 50.0
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    matcher.setMaximumIterations (_matching_max_iterations); // 50
    matcher.setMaxCorrespondenceDistance (_matching_max_correspondence_distance); //3.0
    matcher.setInputTarget (target_cloud_ptr);
    matcher.setInputSource (input_cloud_ptr);

    Eigen::Matrix4d initial_relative_pose (Eigen::Matrix4d::Identity());
    if (HAVE_GRAPH_INPUT)
    {
      uint pre_query_vertex_id = idx - _file_id_step;
      uint cur_query_vertex_id = idx;
      initial_relative_pose = getGraphPose(optimizerForLoadingInput, pre_query_vertex_id).inverse() * getGraphPose(optimizerForLoadingInput, cur_query_vertex_id);
    }
    else if (HAVE_GPS_INPUT)
    {
      uint pre_query_pcd_file_idx = idx - _file_id_step;
      uint cur_query_pcd_file_idx = idx;
      initial_relative_pose = getGPSPose(pre_query_pcd_file_idx).inverse() * getGPSPose(cur_query_pcd_file_idx);
    }
    // std::cout << "condition: " << _initial_poses.size() << " v.s. " << (1 + std::floor((idx-_file_start_id)/_file_id_step)) << std::endl;
    // if ( _initial_poses.size() == file_names.size() )
    // {
    //   initial_pose = (_initial_poses[idx-_file_id_step].inverse() * _initial_poses[idx] ).cast<float>();
    // }
    // else if ( _initial_poses.size() == (1 + std::floor((_file_end_id-_file_start_id)/_file_id_step)) )
    // {
    //   initial_pose = (_initial_poses[std::floor((idx-_file_id_step-_file_start_id)/_file_id_step)].inverse() * _initial_poses[std::floor((idx-_file_start_id)/_file_id_step)] ).cast<float>();
    // }
    // Eigen::Matrix4f initial_pose = (_initial_poses.empty()) ? Eigen::Matrix4f::Identity() : (Eigen::Matrix4f)(_initial_poses[idx-_file_id_step].inverse() * _initial_poses[idx] ).cast<float>();
    matcher.align (*output_cloud_ptr, initial_relative_pose.cast<float>()/*(_initial_poses.empty())?Eigen::Matrix4f::Identity():(Eigen::Matrix4f)(_initial_poses[idx-_file_id_step].inverse()*_initial_poses[idx]).cast<float>()*/);
    Eigen::Matrix4d t = matcher.getFinalTransformation ().cast<double>();
    bool converged = matcher.hasConverged ();
    float matchingScore = matcher.getFitnessScore ();

    transformations.emplace_back(t);

    std::cout << "target_cloud_ptr->size() " << target_cloud_ptr->size() << std::endl;
    std::cout << "input_cloud_ptr->size() " << input_cloud_ptr->size() << std::endl;

    pcl::transformPointCloud(*input_cloud_ptr, *intermediate_cloud_ptr, initial_relative_pose.cast<float>());

    std::cout << "\n Scan-matching has converged:" << converged << " score: " << matchingScore << std::endl;
    std::cout << "\n" << "Transformation Matrix: \n" << t/*ndt.getFinalTransformation()*/ << std::endl;

    // if (_enable_visualization)
    // {
    //   boost::shared_ptr<pcl::visualization::PCLVisualizer> debug_viewer
    //       (new pcl::visualization::PCLVisualizer ("Debug Viewer"));
    //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    //       debug_target_color (target_cloud_ptr, 255, 0, 0);
    //   debug_viewer->addPointCloud<pcl::PointXYZ> (target_cloud_ptr, debug_target_color, "debug target cloud");
    //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    //       debug_output_color (output_cloud_ptr, 0, 255, 0);
    //   debug_viewer->addPointCloud<pcl::PointXYZ> (output_cloud_ptr, debug_output_color, "debug output cloud");
    //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    //       debug_imtermediate_color (intermediate_cloud_ptr, 155, 155, 155);
    //   debug_viewer->addPointCloud<pcl::PointXYZ> (intermediate_cloud_ptr, debug_imtermediate_color, "debug intermediate cloud");
    // }

    if (_enable_visualization)
    {
      Eigen::Matrix<double,4,1> centroid;
      pcl::compute3DCentroid(*pre_cloud_ptr, centroid);
      // Eigen::AngleAxisf init_rotation (1.0, Eigen::Vector3f::UnitY ());
      // Eigen::Translation3f init_translation (centroid[0]-15.0, centroid[1], centroid[2]+15.0);
      // Eigen::Matrix4f extrinsic = (init_translation * init_rotation).matrix ();
      // viewer->setCameraParameters(intrinsic, extrinsic);
      viewer->setCameraPosition(centroid[0]-140.0, centroid[1], centroid[2]+120.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);
      //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
      //viewer->updatePointCloud<pcl::PointXYZI>(whole_cloud_out, intensity_distribution, "whole cloud");
      //viewer->removeAllPointClouds();
      //viewer->addPointCloud(wholeCloud);

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_color (target_cloud_ptr, 255, 0, 0);
      viewer->updatePointCloud<pcl::PointXYZ> (target_cloud_ptr, target_color, "target cloud");
      //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      //  1, "target cloud");

      // Coloring and visualizing transformed input cloud (green).
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        output_color (output_cloud_ptr, 0, 255, 0);
      viewer->updatePointCloud<pcl::PointXYZ> (output_cloud_ptr, output_color, "output cloud");
      //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      //  1, "output cloud");

      // Coloring and visualizing initial transformed input cloud (gray).
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        temp_color (intermediate_cloud_ptr, 155, 155, 155);
      viewer->updatePointCloud<pcl::PointXYZ> (intermediate_cloud_ptr, temp_color, "temp cloud");
      //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      //  1, "temp cloud");

      if (_pause_to_debug)
        viewer->spin();
      else
        viewer->spinOnce();
    }

    t_total = t_total * t;
    pcl::PointCloud<PointTypeIO>::Ptr global_output_cloud_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::transformPointCloud (*tmp_cloud_ptr/*tmp_no_ground_ptr*/, *global_output_cloud_ptr, t_total);

    pre_cloud_ptr->swap(*tmp_cloud_ptr);
    pre_no_ground_ptr->swap(*tmp_no_ground_ptr);
    pre_ground_ptr->swap(*tmp_ground_ptr);
    printf(" ++++ \n");

    Eigen::Matrix4d initial_pose = t_total;
    // std::cout << "condition: " << _initial_poses.size() << " v.s. " << (1 + std::floor((idx-_file_start_id)/_file_id_step)) << std::endl;
    if (HAVE_GRAPH_INPUT)
    {
      uint query_vertex_id = idx;
      initial_pose = getGraphPose(optimizerForLoadingInput, query_vertex_id);
    }
    else if (HAVE_GPS_INPUT)
    {
      uint query_pcd_file_idx = idx;
      initial_pose = getGPSPose(query_pcd_file_idx);
    }

    addGraphVertex(graphOptimizer, idx, initial_pose);
    if (!HAVE_PENDING_GRAPH)
      addGraphVertex(pendingGraphOptimizer, idx, initial_pose);

    float s = matchingScore/1000.0;
    float trans_e = _translation_error_default + _translation_error_factor * s;
    float rot_e = _rotation_error_default + _rotation_error_factor * s;
    std::vector<float> std_var_vec = {trans_e, trans_e, trans_e, rot_e*0.01745, rot_e*0.01745, rot_e*0.01745};
    addGraphEdge(graphOptimizer, pre_vex_id, idx, t, std_var_vec);
    if (_add_initial_alignment_to_graph)
      addGraphEdge(pendingGraphOptimizer, pre_vex_id, idx, t, std_var_vec);

    pre_vex_id = idx;

    if (_save_intial_aligned_data)
    {
      // std::cout << "output point cloud: " << ofname << std::endl;
      // pcl::io::savePCDFileASCII (ofname, *global_output_cloud_ptr);
    }

    if (!output_graph_file_name.empty())
    {
      pendingGraphOptimizer.save(output_graph_file_name.c_str());
      std::cout << "graph saved: " << output_graph_file_name << std::endl;
    }
    //  wholeCloud->clear();
    //}
  }

  if (HAVE_GPS_INPUT)
  {
    for (uint idx = _file_start_id;
        ( _file_id_step > 0 && idx <= _file_end_id ) || ( _file_id_step < 0 && idx >= _file_end_id );
        idx = idx + _file_id_step)
    {
      addGraphVertex(graphOptimizer, idx + _gps_vertex_id_offset, getGPSPose(idx), true);
      if (_add_initial_alignment_to_graph)
        addGraphVertex(pendingGraphOptimizer, idx + _gps_vertex_id_offset, getGPSPose(idx), true);

//      float s = std::pow(matchingScore, 0.25);
//      float trans_e = s*_translation_error_factor;
//      float rot_e = s*_rotation_error_factor;
      std::vector<float> std_var_vec = {0.1, 0.1, 10.0, 3.0*0.01745, 3.0*0.01745, 0.3*0.01745};
      addGraphEdge(graphOptimizer, idx + _gps_vertex_id_offset, idx, Eigen::Matrix4d::Identity(), std_var_vec);
      if (_add_initial_alignment_to_graph)
        addGraphEdge(pendingGraphOptimizer, idx + _gps_vertex_id_offset, idx, Eigen::Matrix4d::Identity(), std_var_vec);
    }
    if (!output_graph_file_name.empty())
    {
      pendingGraphOptimizer.save(output_graph_file_name.c_str());
      std::cout << "graph saved: " << output_graph_file_name << std::endl;
    }
  }
}

void
  refine (std::vector<std::string>& files)
{
  if ( _refine_num_neighbor_to_consider < 2 )
  {
    std::cerr << "num_neighbor_to_consider < 2. Refinement is canceled." << std::endl;
    return;
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Refinement Viewer"));

  if (_enable_visualization)
  {
    viewer->setBackgroundColor (0, 0, 0);
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
    //viewer->addPointCloud<pcl::PointXYZI> (whole_cloud_out, intensity_distribution, "whole cloud");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");
    //viewer->addCoordinateSystem (1.0);
    //viewer->initCameraParameters ();

    pcl::PointCloud<pcl::PointXYZ>::Ptr null_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      target_color (null_cloud_ptr, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      output_color (null_cloud_ptr, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, output_color, "output cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "output cloud");

    // Coloring and visualizing initial transformed input cloud (gray).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      temp_color (null_cloud_ptr, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, temp_color, "temp cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "temp cloud");
  }

  for ( uint target_idx = (_file_id_step > 0) ? _file_start_id : _file_end_id;
        ( _file_id_step > 0 && target_idx <= (_file_end_id - 2*_file_id_step) ) || ( _file_id_step < 0 && target_idx >= (_file_start_id - 2*_file_id_step) );
        target_idx = target_idx + _file_id_step )
  {
    pcl::PointCloud<PointTypeIO>::Ptr pre_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
    loadPCDFile(files[target_idx], pre_cloud_ptr);

    pcl::PointCloud<PointTypeIO>::Ptr pre_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<PointTypeIO>::Ptr pre_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    remove_ground(pre_cloud_ptr, pre_no_ground_ptr, pre_ground_ptr);

    std::vector<pcl::PointCloud<PointTypeFull>::Ptr> segmented_component_ptrs;
    pcl::PointCloud<PointTypeFull>::Ptr pre_useful_cloud_ptr (new pcl::PointCloud<PointTypeFull>);
    pcl::PointCloud<PointTypeFull>::Ptr segmented_cloud_to_show_ptr (new pcl::PointCloud<PointTypeFull>);
    segment_cloud(pre_no_ground_ptr, segmented_component_ptrs, pre_useful_cloud_ptr, segmented_cloud_to_show_ptr);

    Eigen::Matrix4d t_total (Eigen::Matrix4d::Identity ());


    uint end_idx;
    if ( _file_id_step==0 )
    {
      std::cerr << "file_id_step is 0. Refinement is canceled." << std::endl;
      return;
    }
    else if (_file_id_step>0)
    {
      end_idx = ((target_idx + _refine_num_neighbor_to_consider*_file_id_step) <= _file_end_id) ? (target_idx + _refine_num_neighbor_to_consider*_file_id_step) : _file_end_id;
    }
    else if (_file_id_step<0)
    {
      end_idx = ((target_idx + _refine_num_neighbor_to_consider*_file_id_step) >= _file_start_id) ? (target_idx + _refine_num_neighbor_to_consider*_file_id_step) : _file_start_id;
    }

    for ( uint idx = target_idx + 2*_file_id_step;
          ( _file_id_step > 0 && idx <= end_idx ) || ( _file_id_step < 0 && idx >= end_idx );
          idx = idx + _file_id_step )
    {
      // pcl::console::print_color(std::cout, pcl::console::TT_RESET, TT_YELLOW, ")
      std::cout << " * * * * * prev_id: " << target_idx << " * * * * * cur_id: " << idx << std::endl;

      pcl::PointCloud<PointTypeIO>::Ptr tmp_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
      loadPCDFile(files[idx], tmp_cloud_ptr);

      Eigen::Matrix4d initial_relative_pose (Eigen::Matrix4d::Identity());
      if (HAVE_GRAPH_INPUT)
      {
        uint pre_query_vertex_id = target_idx;
        uint cur_query_vertex_id = idx;
        initial_relative_pose = getGraphPose(optimizerForLoadingInput, pre_query_vertex_id).inverse() * getGraphPose(optimizerForLoadingInput, cur_query_vertex_id);
      }
      else if (HAVE_GPS_INPUT)
      {
        uint pre_query_pcd_file_idx = target_idx;
        uint cur_query_pcd_file_idx = idx;
        initial_relative_pose = getGPSPose(pre_query_pcd_file_idx).inverse() * getGPSPose(cur_query_pcd_file_idx);
      }
      else
      {
        std::cout << "Relative transformation chains: " << std::endl;
        for (uint trans_id = target_idx; trans_id < idx; trans_id = trans_id + _file_id_step )
        {
          initial_relative_pose = initial_relative_pose * transformations[std::floor((trans_id-_file_start_id)/_file_id_step)];
          std::cout << files[trans_id] << " <--- " << files[trans_id+_file_id_step] << " * " << std::endl;
        }
        std::cout << "\n" << "Previous Transformation Matrix: \n" << initial_relative_pose/*ndt.getFinalTransformation()*/ << std::endl;
      }

      // std::cout << "debug 2 " <<tmp_cloud_ptr->size() << std::endl;
      pcl::PointCloud<PointTypeIO>::Ptr tmp_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
      pcl::PointCloud<PointTypeIO>::Ptr tmp_ground_ptr (new pcl::PointCloud<PointTypeIO>);
      remove_ground(tmp_cloud_ptr, tmp_no_ground_ptr, tmp_ground_ptr);

      // std::vector<pcl::PointCloud<PointTypeFull>::Ptr> segmented_component_ptrs;
      pcl::PointCloud<PointTypeFull>::Ptr tmp_useful_cloud_ptr (new pcl::PointCloud<PointTypeFull>);
      // pcl::PointCloud<PointTypeFull>::Ptr segmented_cloud_to_show_ptr (new pcl::PointCloud<PointTypeFull>);
      segment_cloud(tmp_no_ground_ptr, segmented_component_ptrs, tmp_useful_cloud_ptr, segmented_cloud_to_show_ptr);

      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::copyPointCloud(*tmp_no_ground_ptr/*tmp_useful_cloud_ptr*/,*input_cloud_ptr);
      // pcl::copyPointCloud(*pre_no_ground_ptr/*pre_useful_cloud_ptr*/,*target_cloud_ptr);

      pcl::copyPointCloud(*tmp_no_ground_ptr/*+*tmp_ground_ptr*/, *input_cloud_ptr);
      pcl::copyPointCloud(*pre_no_ground_ptr/*+*pre_ground_ptr*/, *target_cloud_ptr);

      pcl::transformPointCloud (*input_cloud_ptr/*tmp_no_ground_ptr*/, *intermediate_cloud_ptr, initial_relative_pose.cast<float>());

      ///////////////////////////////////////////////////////////////////////////////////////////////////
      // pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> matcher;
      // boost::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> > warp_fcn
      //   (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);
      // boost::shared_ptr<pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> > te
      //   (new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>);
      // te->setWarpFunction (warp_fcn);
      // matcher.setTransformationEstimation (te);
      // matcher.setRANSACOutlierRejectionThreshold (_matching_outlier_threshold);
      ///////////////////////////////////////////////////////////////////////////////////////////////////
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> matcher;
      ///////////////////////////////////////////////////////////////////////////////////////////////////
      // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> matcher;
      // matcher.setTransformationEpsilon (_matching_transformation_epsilon); // 0.1 5.0
      // matcher.setStepSize (_matching_step_size); // 0.1 5.0
      // matcher.setResolution (_matching_resolution); // 0.5 50.0
      ///////////////////////////////////////////////////////////////////////////////////////////////////
      matcher.setMaximumIterations (_matching_max_iterations);
      matcher.setMaxCorrespondenceDistance (_matching_max_correspondence_distance);
      matcher.setInputTarget (target_cloud_ptr);
      matcher.setInputSource (input_cloud_ptr);
      matcher.align (*output_cloud_ptr, initial_relative_pose.cast<float>());
      Eigen::Matrix4d t = matcher.getFinalTransformation().cast<double>();
      bool converged = matcher.hasConverged ();
      float matchingScore = matcher.getFitnessScore ();

      std::cout << "target_cloud_ptr->size() " << target_cloud_ptr->size() << std::endl;
      std::cout << "input_cloud_ptr->size() " << input_cloud_ptr->size() << std::endl;

      std::cout << "\n Scan-matching has converged:" << converged << " score: " << matchingScore << std::endl;
      std::cout << "\n" << "Current Transformation Matrix: \n" << t/*ndt.getFinalTransformation()*/ << std::endl;

      if (_enable_visualization)
      {
        Eigen::Matrix<double,4,1> centroid;
        pcl::compute3DCentroid(*pre_cloud_ptr, centroid);
        // Eigen::AngleAxisf init_rotation (1.0, Eigen::Vector3f::UnitY ());
        // Eigen::Translation3f init_translation (centroid[0]-15.0, centroid[1], centroid[2]+15.0);
        // extrinsic = (init_translation * init_rotation).matrix ();
        // viewer->setCameraParameters(intrinsic, extrinsic);
        viewer->setCameraPosition(centroid[0]-140.0, centroid[1], centroid[2]+120.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);
        //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
        //viewer->updatePointCloud<pcl::PointXYZI>(whole_cloud_out, intensity_distribution, "whole cloud");
        //viewer->removeAllPointClouds();
        //viewer->addPointCloud(wholeCloud);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          target_color (target_cloud_ptr, 255, 0, 0);
        viewer->updatePointCloud<pcl::PointXYZ> (target_cloud_ptr, target_color, "target cloud");
        //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        //  1, "target cloud");

        // Coloring and visualizing transformed input cloud (green).
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          output_color (output_cloud_ptr, 0, 255, 0);
        viewer->updatePointCloud<pcl::PointXYZ> (output_cloud_ptr, output_color, "output cloud");
        //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        //  1, "output cloud");

        // Coloring and visualizing initial transformed input cloud (gray).
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          temp_color (intermediate_cloud_ptr, 155, 155, 155);
        viewer->updatePointCloud<pcl::PointXYZ> (intermediate_cloud_ptr, temp_color, "temp cloud");
        //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        //  1, "temp cloud");

        if (_pause_to_debug)
          viewer->spin();
        else
          viewer->spinOnce();
      }

      // t_total = t_total * t;
      // pcl::PointCloud<PointTypeIO>::Ptr global_output_cloud_ptr (new pcl::PointCloud<PointTypeIO>);
      // pcl::transformPointCloud (*tmp_cloud_ptr/*tmp_no_ground_ptr*/, *global_output_cloud_ptr, t_total);

      // pre_cloud_ptr->swap(*tmp_cloud_ptr);
      // pre_no_ground_ptr->swap(*tmp_no_ground_ptr);
      // pre_ground_ptr->swap(*tmp_ground_ptr);
      printf(" ---- \n");

      // g2o::VertexSE3* v = new g2o::VertexSE3;
      // v->setId(idx);
      // Eigen::Isometry3d iso3;
      // iso3 = t_total.block<3,3>(0,0).cast<double>();
      // iso3.translation() = t_total.block<3,1>(0,3).cast<double>();
      // v->setEstimate(iso3);
      // vertices.push_back(v);
      // std::cout << "[vertex " << idx << "] ";
      // v->write(std::cout);
      // std::cout << std::endl;

      float s = matchingScore/1000.0;
      float trans_e = _translation_error_default + _translation_error_factor * s;
      float rot_e = _rotation_error_default + _rotation_error_factor * s;
      std::vector<float> std_var_vec = {trans_e, trans_e, trans_e, rot_e*0.01745, rot_e*0.01745, rot_e*0.01745};
      addGraphEdge(graphOptimizer, target_idx, idx, t, std_var_vec);
      if (_add_refinement_to_graph)
        addGraphEdge(pendingGraphOptimizer, target_idx, idx, t, std_var_vec);

      // std::cout << "output point cloud: " << ofname << std::endl;
      // pcl::io::savePCDFileASCII (ofname, *global_output_cloud_ptr);

      // wholeCloud->clear();
      // }

      if (!output_graph_file_name.empty())
      {
        pendingGraphOptimizer.save(output_graph_file_name.c_str());
        std::cout << "graph saved: " << output_graph_file_name << std::endl;
      }
    }
  }
}

void
  evaluate_edge (std::vector<std::string>& files, int idx1, int idx2)
{

//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Close Loop Viewer"));
//
//  if (_enable_visualization)
//  {
//    viewer->setBackgroundColor (0, 0, 0);
//    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
//    //viewer->addPointCloud<pcl::PointXYZI> (whole_cloud_out, intensity_distribution, "whole cloud");
//    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");
//    //viewer->addCoordinateSystem (1.0);
//    //viewer->initCameraParameters ();
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr null_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//      target_color (null_cloud_ptr, 255, 0, 0);
//    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, target_color, "target cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//      1, "target cloud");
//
//    // Coloring and visualizing transformed input cloud (green).
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//      output_color (null_cloud_ptr, 0, 255, 0);
//    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, output_color, "output cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//      1, "output cloud");
//
//    // Coloring and visualizing initial transformed input cloud (gray).
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//      temp_color (null_cloud_ptr, 0, 0, 255);
//    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, temp_color, "temp cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//      1, "temp cloud");
//  }

    pcl::PointCloud<PointTypeIO>::Ptr pre_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
    loadPCDFile(files[idx1], pre_cloud_ptr);

    pcl::PointCloud<PointTypeIO>::Ptr pre_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<PointTypeIO>::Ptr pre_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    remove_ground(pre_cloud_ptr, pre_no_ground_ptr, pre_ground_ptr);

    std::vector<pcl::PointCloud<PointTypeFull>::Ptr> segmented_component_ptrs;
    pcl::PointCloud<PointTypeFull>::Ptr pre_useful_cloud_ptr (new pcl::PointCloud<PointTypeFull>);
    pcl::PointCloud<PointTypeFull>::Ptr segmented_cloud_to_show_ptr (new pcl::PointCloud<PointTypeFull>);
    segment_cloud(pre_no_ground_ptr, segmented_component_ptrs, pre_useful_cloud_ptr, segmented_cloud_to_show_ptr);



      // pcl::console::print_color(std::cout, pcl::console::TT_RESET, TT_YELLOW, ")
      std::cout << " * * * * * prev_id: " << idx1 << " * * * * * cur_id: " << idx2 << std::endl;

      pcl::PointCloud<PointTypeIO>::Ptr tmp_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
      loadPCDFile(files[idx2], tmp_cloud_ptr);

      pcl::PointXYZ p1, p2;
      Eigen::Matrix4d pre_pose (Eigen::Matrix4d::Identity());
      Eigen::Matrix4d cur_pose (Eigen::Matrix4d::Identity());

      Eigen::Matrix4d initial_relative_pose (Eigen::Matrix4d::Identity());
      if (HAVE_GRAPH_INPUT)
      {
        uint pre_query_vertex_id = idx1;
        uint cur_query_vertex_id = idx2;
        initial_relative_pose = getGraphPose(optimizerForLoadingInput, pre_query_vertex_id).inverse() * getGraphPose(optimizerForLoadingInput, cur_query_vertex_id);
        pre_pose = getGraphPose(optimizerForLoadingInput, pre_query_vertex_id);
        cur_pose = getGraphPose(optimizerForLoadingInput, cur_query_vertex_id);
        p1 = pcl::PointXYZ(getGraphPose(optimizerForLoadingInput, pre_query_vertex_id).cast<float>()(0,3),getGraphPose(optimizerForLoadingInput, pre_query_vertex_id).cast<float>()(1,3),getGraphPose(optimizerForLoadingInput, pre_query_vertex_id).cast<float>()(2,3));
        p2 = pcl::PointXYZ(getGraphPose(optimizerForLoadingInput, cur_query_vertex_id).cast<float>()(0,3),getGraphPose(optimizerForLoadingInput, cur_query_vertex_id).cast<float>()(1,3),getGraphPose(optimizerForLoadingInput, cur_query_vertex_id).cast<float>()(2,3));
      }
      else if (HAVE_GPS_INPUT)
      {
        uint pre_query_pcd_file_idx = idx1;
        uint cur_query_pcd_file_idx = idx2;
        initial_relative_pose = getGPSPose(pre_query_pcd_file_idx).inverse() * getGPSPose(cur_query_pcd_file_idx);
      }
      else
      {
        std::cout << "Relative transformation chains: " << std::endl;
        for (uint trans_id = idx1; trans_id < idx2; trans_id = trans_id + _file_id_step )
        {
          initial_relative_pose = initial_relative_pose * transformations[std::floor((trans_id-_file_start_id)/_file_id_step)];
          std::cout << files[trans_id] << " <--- " << files[trans_id+_file_id_step] << " * " << std::endl;
        }
        std::cout << "\n" << "Previous Transformation Matrix: \n" << initial_relative_pose/*ndt.getFinalTransformation()*/ << std::endl;
      }

      // std::cout << "debug 2 " <<tmp_cloud_ptr->size() << std::endl;
      pcl::PointCloud<PointTypeIO>::Ptr tmp_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
      pcl::PointCloud<PointTypeIO>::Ptr tmp_ground_ptr (new pcl::PointCloud<PointTypeIO>);
      remove_ground(tmp_cloud_ptr, tmp_no_ground_ptr, tmp_ground_ptr);

      // std::vector<pcl::PointCloud<PointTypeFull>::Ptr> segmented_component_ptrs;
      pcl::PointCloud<PointTypeFull>::Ptr tmp_useful_cloud_ptr (new pcl::PointCloud<PointTypeFull>);
      // pcl::PointCloud<PointTypeFull>::Ptr segmented_cloud_to_show_ptr (new pcl::PointCloud<PointTypeFull>);
      segment_cloud(tmp_no_ground_ptr, segmented_component_ptrs, tmp_useful_cloud_ptr, segmented_cloud_to_show_ptr);

      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::copyPointCloud(*tmp_no_ground_ptr/*tmp_useful_cloud_ptr*/,*input_cloud_ptr);
      // pcl::copyPointCloud(*pre_no_ground_ptr/*pre_useful_cloud_ptr*/,*target_cloud_ptr);
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_output_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

      pcl::copyPointCloud(*tmp_useful_cloud_ptr/*+*tmp_ground_ptr*/, *input_cloud_ptr);
      pcl::copyPointCloud(*pre_useful_cloud_ptr/*+*pre_ground_ptr*/, *target_cloud_ptr);

      pcl::transformPointCloud ( *input_cloud_ptr, *temp_input_cloud_ptr, cur_pose.cast<float>());
      pcl::transformPointCloud ( *target_cloud_ptr, *temp_target_cloud_ptr, pre_pose.cast<float>());
      std::cout << "\n" << "input_cloud_ptr Transformation Matrix: \n" << cur_pose.cast<float>() << std::endl;
      std::cout << "\n" << "target_cloud_ptr Transformation Matrix: \n" << pre_pose.cast<float>() << std::endl;

//      pcl::transformPointCloud (*input_cloud_ptr/*tmp_no_ground_ptr*/, *intermediate_cloud_ptr, initial_relative_pose.cast<float>());

      ///////////////////////////////////////////////////////////////////////////////////////////////////
      // pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> matcher;
      // boost::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> > warp_fcn
      //   (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);
      // boost::shared_ptr<pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> > te
      //   (new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>);
      // te->setWarpFunction (warp_fcn);
      // matcher.setTransformationEstimation (te);
      // matcher.setRANSACOutlierRejectionThreshold (_matching_outlier_threshold);
      ///////////////////////////////////////////////////////////////////////////////////////////////////
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> matcher;
      ///////////////////////////////////////////////////////////////////////////////////////////////////
      // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> matcher;
      // matcher.setTransformationEpsilon (_matching_transformation_epsilon); // 0.1 5.0
      // matcher.setStepSize (_matching_step_size); // 0.1 5.0
      // matcher.setResolution (_matching_resolution); // 0.5 50.0
      ///////////////////////////////////////////////////////////////////////////////////////////////////
      matcher.setMaximumIterations (_matching_max_iterations);
      matcher.setMaxCorrespondenceDistance (_closeloop_max_correspondence_distance);
      matcher.setInputTarget (target_cloud_ptr);
      matcher.setInputSource (input_cloud_ptr);
      matcher.align (*output_cloud_ptr, initial_relative_pose.cast<float>());
      Eigen::Matrix4d t = matcher.getFinalTransformation().cast<double>();
      bool converged = matcher.hasConverged ();
      float matchingScore = matcher.getFitnessScore ();

      pcl::transformPointCloud ( *input_cloud_ptr, *temp_output_cloud_ptr, pre_pose.cast<float>()*t.cast<float>());

      std::cout << "target_cloud_ptr->size() " << target_cloud_ptr->size() << std::endl;
      std::cout << "input_cloud_ptr->size() " << input_cloud_ptr->size() << std::endl;

      std::cout << "\n Scan-matching has converged:" << converged << " score: " << matchingScore << std::endl;
      std::cout << "\n" << "Current Transformation Matrix: \n" << t/*ndt.getFinalTransformation()*/ << std::endl;

      if (_enable_visualization)
      {
//        Eigen::Matrix<double,4,1> centroid;
//        pcl::compute3DCentroid(*pre_cloud_ptr, centroid);
        // Eigen::AngleAxisf init_rotation (1.0, Eigen::Vector3f::UnitY ());
        // Eigen::Translation3f init_translation (centroid[0]-15.0, centroid[1], centroid[2]+15.0);
        // extrinsic = (init_translation * init_rotation).matrix ();
        // viewer->setCameraParameters(intrinsic, extrinsic);
//        viewer->setCameraPosition(centroid[0]-140.0, centroid[1], centroid[2]+120.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);
        //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
        //viewer->updatePointCloud<pcl::PointXYZI>(whole_cloud_out, intensity_distribution, "whole cloud");
        //viewer->removeAllPointClouds();
        //viewer->addPointCloud(wholeCloud);

        float line_color_factor = matchingScore > 400.0 ? 1.0 : (matchingScore < 100.0 ? 0.0 : (matchingScore-100.0)/400.0);
        std::cout << "line_color_factor: " << line_color_factor << std::endl;
        char line_id[10];
        sprintf(line_id, "line_%d-%d", idx1, idx2);
        std::string line_name(line_id);
        viewer->addLine (p1, p2, 1.0-line_color_factor, line_color_factor, 0.0,line_name);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          target_color (temp_target_cloud_ptr, 255, 0, 0);
        viewer->updatePointCloud<pcl::PointXYZ> (temp_target_cloud_ptr, target_color, "target cloud");
        //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        //  1, "target cloud");

        // Coloring and visualizing transformed input cloud (green).
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          output_color (temp_output_cloud_ptr, 0, 255, 0);
        viewer->updatePointCloud<pcl::PointXYZ> (temp_output_cloud_ptr, output_color, "output cloud");
        //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        //  1, "output cloud");

        // Coloring and visualizing initial transformed input cloud (gray).
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          temp_color (temp_input_cloud_ptr, 155, 155, 155);
        viewer->updatePointCloud<pcl::PointXYZ> (temp_input_cloud_ptr, temp_color, "temp cloud");
        //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        //  1, "temp cloud");

        if (_pause_to_debug)
          viewer->spin();
        else
          viewer->spinOnce();
      }

      // t_total = t_total * t;
      // pcl::PointCloud<PointTypeIO>::Ptr global_output_cloud_ptr (new pcl::PointCloud<PointTypeIO>);
      // pcl::transformPointCloud (*tmp_cloud_ptr/*tmp_no_ground_ptr*/, *global_output_cloud_ptr, t_total);

      // pre_cloud_ptr->swap(*tmp_cloud_ptr);
      // pre_no_ground_ptr->swap(*tmp_no_ground_ptr);
      // pre_ground_ptr->swap(*tmp_ground_ptr);
      printf(" ---- \n");

      // g2o::VertexSE3* v = new g2o::VertexSE3;
      // v->setId(idx);
      // Eigen::Isometry3d iso3;
      // iso3 = t_total.block<3,3>(0,0).cast<double>();
      // iso3.translation() = t_total.block<3,1>(0,3).cast<double>();
      // v->setEstimate(iso3);
      // vertices.push_back(v);
      // std::cout << "[vertex " << idx << "] ";
      // v->write(std::cout);
      // std::cout << std::endl;

      float s = matchingScore/1000.0;
      float trans_e = _translation_error_default + _translation_error_factor * s;
      float rot_e = _rotation_error_default + _rotation_error_factor * s;
      std::vector<float> std_var_vec = {trans_e, trans_e, trans_e, rot_e*0.01745, rot_e*0.01745, rot_e*0.01745};
      std::cout << "Test 1 "  << std::endl;
      addGraphEdge(graphOptimizer, idx1, idx2, t, std_var_vec);
      if (_add_loop_closing_to_graph)
        addGraphEdge(pendingGraphOptimizer, idx1, idx2, t, std_var_vec);

      if (!output_graph_file_name.empty())
      {
        pendingGraphOptimizer.save(output_graph_file_name.c_str());
        std::cout << "graph saved: " << output_graph_file_name << std::endl;
      }
      // std::cout << "output point cloud: " << ofname << std::endl;
      // pcl::io::savePCDFileASCII (ofname, *global_output_cloud_ptr);

      // wholeCloud->clear();
      // }
}

void
  close_loop (std::vector<std::string>& files)
{
  if ( _refine_num_neighbor_to_consider < 2 )
  {
    std::cerr << "num_neighbor_to_consider < 2. Refinement is canceled." << std::endl;
    return;
  }

//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Loop Closing Viewer"));

  if (_enable_visualization)
  {
    viewer->setBackgroundColor (0, 0, 0);
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
    //viewer->addPointCloud<pcl::PointXYZI> (whole_cloud_out, intensity_distribution, "whole cloud");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");
    //viewer->addCoordinateSystem (1.0);
    //viewer->initCameraParameters ();

    pcl::PointCloud<pcl::PointXYZ>::Ptr null_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      target_color (null_cloud_ptr, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      output_color (null_cloud_ptr, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, output_color, "output cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "output cloud");

    // Coloring and visualizing initial transformed input cloud (gray).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      temp_color (null_cloud_ptr, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, temp_color, "temp cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "temp cloud");

    for ( uint idx = (_file_id_step > 0) ? _file_start_id : _file_end_id;
          ( _file_id_step > 0 && idx <= _file_end_id ) || ( _file_id_step < 0 && idx >= _file_start_id );
          idx = idx + _file_id_step )
    {
      Eigen::Matrix4d initial_pose (Eigen::Matrix4d::Identity());
      if (HAVE_GRAPH_INPUT)
      {
        uint cur_query_vertex_id = idx;
        initial_pose = getGraphPose(optimizerForLoadingInput, cur_query_vertex_id);
      }
      else if (HAVE_GPS_INPUT)
      {
        uint cur_query_pcd_file_idx = idx;
        initial_pose = getGPSPose(cur_query_pcd_file_idx);
      }
      Eigen::Transform<float, 3, Eigen::Affine> pse_pose(initial_pose.cast<float>());
      //std::cout << pse_pose.translation()[0] << " " << pse_pose.translation()[1] << std::endl;
      // Eigen::Quaternion<float> q(pse_pose.rotation());
      //std::cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      viewer->addCoordinateSystem (1.0, pse_pose);
    }

  }

  for ( uint target_idx = (_file_id_step > 0) ? _file_start_id : _file_end_id;
        ( _file_id_step > 0 && target_idx <= (_file_end_id - _refine_num_neighbor_to_consider*_file_id_step - _file_id_step) ) || ( _file_id_step < 0 && target_idx >= (_file_start_id - _refine_num_neighbor_to_consider*_file_id_step - _file_id_step) );
        target_idx = target_idx + _file_id_step )
  {
    std::cout << ": ";
//    pcl::PointCloud<PointTypeIO>::Ptr pre_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
//    loadPCDFile(files[target_idx], pre_cloud_ptr);
//
//    pcl::PointCloud<PointTypeIO>::Ptr pre_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
//    pcl::PointCloud<PointTypeIO>::Ptr pre_ground_ptr (new pcl::PointCloud<PointTypeIO>);
//    remove_ground(pre_cloud_ptr, pre_no_ground_ptr, pre_ground_ptr);
//
//    std::vector<pcl::PointCloud<PointTypeFull>::Ptr> segmented_component_ptrs;
//    pcl::PointCloud<PointTypeFull>::Ptr pre_useful_cloud_ptr (new pcl::PointCloud<PointTypeFull>);
//    pcl::PointCloud<PointTypeFull>::Ptr segmented_cloud_to_show_ptr (new pcl::PointCloud<PointTypeFull>);
//    segment_cloud(pre_no_ground_ptr, segmented_component_ptrs, pre_useful_cloud_ptr, segmented_cloud_to_show_ptr);

    Eigen::Matrix4d t_total (Eigen::Matrix4d::Identity ());

//    uint end_idx;
//    if ( _file_id_step==0 )
//    {
//      std::cerr << "file_id_step is 0. Refinement is canceled." << std::endl;
//      return;
//    }
//    else if (_file_id_step>0)
//    {
//      end_idx = target_idx + _refine_num_neighbor_to_consider*_file_id_step   <= _file_end_id ? target_idx + _refine_num_neighbor_to_consider*_file_id_step : _file_end_id;
//    }
//    else if (_file_id_step<0)
//    {
//      end_idx = target_idx + _refine_num_neighbor_to_consider*_file_id_step  >= _file_start_id ? target_idx + _refine_num_neighbor_to_consider*_file_id_step : _file_start_id;
//    }

    bool disappeared = false;
    bool detection_enabled = false;
    uint closeloop_pose_idx = target_idx;
    double minimun_dist = std::numeric_limits<double>::max();

    for ( uint idx = target_idx + _refine_num_neighbor_to_consider*_file_id_step + _file_id_step;
          ( _file_id_step > 0 && idx <= _file_end_id ) || ( _file_id_step < 0 && idx >= _file_start_id );
          idx = idx + _file_id_step )
    {
      // pcl::console::print_color(std::cout, pcl::console::TT_RESET, TT_YELLOW, ")
//      std::cout << " * * * * * prev_id: " << target_idx << " * * * * * cur_id: " << idx ;// << std::endl;

//      pcl::PointCloud<PointTypeIO>::Ptr tmp_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
//      loadPCDFile(files[idx], tmp_cloud_ptr);

      Eigen::Matrix4d initial_relative_pose (Eigen::Matrix4d::Identity());
      if (HAVE_GRAPH_INPUT)
      {
        uint pre_query_vertex_id = target_idx;
        uint cur_query_vertex_id = idx;
        initial_relative_pose = getGraphPose(optimizerForLoadingInput, pre_query_vertex_id).inverse() * getGraphPose(optimizerForLoadingInput, cur_query_vertex_id);
        std::cout << "  GRAPH  " ;//<< std::endl;
      }
      else if (HAVE_GPS_INPUT)
      {
        uint pre_query_pcd_file_idx = target_idx;
        uint cur_query_pcd_file_idx = idx;
        initial_relative_pose = getGPSPose(pre_query_pcd_file_idx).inverse() * getGPSPose(cur_query_pcd_file_idx);
        std::cout << "  GPS  " ;//<< std::endl;
      }
      else
      {
        std::cout << "Relative transformation chains: " << std::endl;
        for (uint trans_id = target_idx; trans_id < idx; trans_id = trans_id + _file_id_step )
        {
          initial_relative_pose = initial_relative_pose * transformations[std::floor((trans_id-_file_start_id)/_file_id_step)];
          std::cout << files[trans_id] << " <--- " << files[trans_id+_file_id_step] << " * " << std::endl;
        }
        std::cout << "\n" << "Previous Transformation Matrix: \n" << initial_relative_pose/*ndt.getFinalTransformation()*/ << std::endl;
      }

      double pose_distance = initial_relative_pose(1,3)*initial_relative_pose(1,3) + initial_relative_pose(2,3)*initial_relative_pose(2,3);
      pose_distance = std::pow(pose_distance, 0.5);
      std::cout << pose_distance;
      if ( !disappeared )
      {
        if ( pose_distance <= _closeloop_allowed_overlap_distance )
        {
          if ( detection_enabled && pose_distance < minimun_dist )
          {
            minimun_dist = pose_distance;
            closeloop_pose_idx = idx;
            std::cout << " 1 ";
          }
        }
        else
        {
          disappeared = true;
          if ( detection_enabled )
          {
            evaluate_edge(files, target_idx, closeloop_pose_idx);
            minimun_dist = std::numeric_limits<double>::max();
            std::cout << " 2 ";
          }
        }
      }
      else
      {
        detection_enabled = true;
        if ( pose_distance <= _closeloop_allowed_overlap_distance )
        {
          disappeared = false;
          minimun_dist = pose_distance;
          closeloop_pose_idx = idx;
          std::cout << " 3 ";
        }
      }
      std::cout << " ; " << std::endl;
    }
  }
}

void
  replay(std::vector<std::string>& files)
{
  viewer->spinOnce(5000);
  uint pre_vex_id = _file_start_id;

  pcl::PointCloud<PointTypeIO>::Ptr pre_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
  loadPCDFile(files[pre_vex_id], pre_cloud_ptr);

  pcl::PointCloud<PointTypeIO>::Ptr pre_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeIO>::Ptr pre_ground_ptr (new pcl::PointCloud<PointTypeIO>);
  remove_ground(pre_cloud_ptr, pre_no_ground_ptr, pre_ground_ptr);

  pcl::PointCloud<PointTypeIO>::Ptr pre_transformed_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
  pcl::transformPointCloud(*pre_no_ground_ptr, *pre_transformed_cloud_ptr, getGraphPose(optimizerForLoadingInput, pre_vex_id).cast<float>());

  char pre_pc_id[10];
  sprintf(pre_pc_id, "pointcloud-%d", pre_vex_id);
  std::string pre_pc_name(pre_pc_id);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(pre_transformed_cloud_ptr, "intensity");
  viewer->addPointCloud<pcl::PointXYZI> (pre_transformed_cloud_ptr, intensity_distribution, pre_pc_name);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, pre_pc_name);

  // int tmp_counter = 0;
  for ( uint idx = (_file_id_step > 0) ? (_file_start_id+_file_id_step) : (_file_end_id-_file_id_step);
      ( _file_id_step > 0 && idx <= _file_end_id ) || ( _file_id_step < 0 && idx >= _file_start_id );
      idx = idx + _file_id_step )
  {
    pcl::PointCloud<PointTypeIO>::Ptr cur_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
    loadPCDFile(files[idx], cur_cloud_ptr);

    pcl::PointCloud<PointTypeIO>::Ptr cur_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<PointTypeIO>::Ptr cur_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    remove_ground(cur_cloud_ptr, cur_no_ground_ptr, cur_ground_ptr);

    pcl::PointCloud<PointTypeIO>::Ptr cur_transformed_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
    pcl::transformPointCloud(*cur_no_ground_ptr, *cur_transformed_cloud_ptr, getGraphPose(optimizerForLoadingInput, idx).cast<float>());

    Eigen::Matrix<double,4,1> centroid;
    pcl::compute3DCentroid(*cur_transformed_cloud_ptr, centroid);
    viewer->setCameraPosition(centroid[0]-140.0, centroid[1], centroid[2]+120.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);

    char cur_pc_id[10];
    sprintf(cur_pc_id, "pointcloud-%d", idx);
    std::string cur_pc_name(cur_pc_id);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(cur_transformed_cloud_ptr, "intensity");
    viewer->addPointCloud<pcl::PointXYZI> (cur_transformed_cloud_ptr, intensity_distribution, cur_pc_name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cur_pc_name);

    Eigen::Matrix4d pre_pose = getGraphPose(optimizerForLoadingInput, pre_vex_id);
    Eigen::Matrix4d cur_pose = getGraphPose(optimizerForLoadingInput, idx);
    pcl::PointXYZ p1 = pcl::PointXYZ(pre_pose.cast<float>()(0,3), pre_pose.cast<float>()(1,3), pre_pose.cast<float>()(2,3));
    pcl::PointXYZ p2 = pcl::PointXYZ(cur_pose.cast<float>()(0,3), cur_pose.cast<float>()(1,3), cur_pose.cast<float>()(2,3));

    float line_color_factor = 0.0;//matchingScore > 400.0 ? 1.0 : (matchingScore < 100.0 ? 0.0 : (matchingScore-100.0)/400.0);
    std::cout << "line_color_factor: " << line_color_factor << std::endl;
    char line_id[10];
    sprintf(line_id, "line_%d-%d", pre_vex_id, idx);
    std::string line_name(line_id);
    viewer->addLine (p1, p2, 1.0/*-line_color_factor*/, 1.0/*line_color_factor*/, 1.0, line_name);

    std::vector<uint> loopclosing_id_vec;
    getLoopClosingPoseIds(pendingGraphOptimizer, idx, loopclosing_id_vec);
    for (uint v_id = 0; v_id < loopclosing_id_vec.size(); v_id++)
    {
      Eigen::Matrix4d closing_pose = getGraphPose(optimizerForLoadingInput, loopclosing_id_vec[v_id]);
      pcl::PointXYZ p3 = pcl::PointXYZ(closing_pose.cast<float>()(0,3), closing_pose.cast<float>()(1,3), closing_pose.cast<float>()(2,3));

      char closing_line_id[10];
      sprintf(closing_line_id, "line_%d-%d", loopclosing_id_vec[v_id], idx);
      std::cout << closing_line_id << std::endl;
      std::string closing_line_name(closing_line_id);
      viewer->addLine (p1, p3, 1.0/*-line_color_factor*/, 1.0/*line_color_factor*/, 1.0, closing_line_name);
    }

    if (_pause_to_debug)
      viewer->spin();
    else
      viewer->spinOnce(800);

    pre_cloud_ptr->swap(*cur_cloud_ptr);
    pre_vex_id = idx;
    printf(" ++++ \n");

  }

  viewer->spinOnce(5000);
  pre_vex_id = _file_start_id;

  pcl::PointCloud<PointTypeIO>::Ptr start_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
  loadPCDFile(files[pre_vex_id], start_cloud_ptr);

  pcl::PointCloud<PointTypeIO>::Ptr start_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeIO>::Ptr start_ground_ptr (new pcl::PointCloud<PointTypeIO>);
  remove_ground(start_cloud_ptr, start_no_ground_ptr, start_ground_ptr);

  pcl::PointCloud<PointTypeIO>::Ptr start_transformed_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
  pcl::transformPointCloud(*start_no_ground_ptr, *start_transformed_cloud_ptr, getGraphPose(pendingGraphOptimizer, _file_start_id).cast<float>());

  char start_pc_id[10];
  sprintf(start_pc_id, "pointcloud-%d", _file_start_id);
  std::string start_pc_name(start_pc_id);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> new_intensity_distribution(start_transformed_cloud_ptr, "intensity");
  viewer->updatePointCloud<pcl::PointXYZI> (start_transformed_cloud_ptr, new_intensity_distribution, start_pc_name);

  // int tmp_counter = 0;
  for ( uint idx = (_file_id_step > 0) ? (_file_start_id+_file_id_step) : (_file_end_id-_file_id_step);
      ( _file_id_step > 0 && idx <= _file_end_id ) || ( _file_id_step < 0 && idx >= _file_start_id );
      idx = idx + _file_id_step )
  {
    pcl::PointCloud<PointTypeIO>::Ptr cur_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
    loadPCDFile(files[idx], cur_cloud_ptr);

    pcl::PointCloud<PointTypeIO>::Ptr cur_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<PointTypeIO>::Ptr cur_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    remove_ground(cur_cloud_ptr, cur_no_ground_ptr, cur_ground_ptr);

    pcl::PointCloud<PointTypeIO>::Ptr cur_transformed_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
    pcl::transformPointCloud(*cur_no_ground_ptr, *cur_transformed_cloud_ptr, getGraphPose(pendingGraphOptimizer, idx).cast<float>());

    Eigen::Matrix<double,4,1> centroid;
    pcl::compute3DCentroid(*cur_transformed_cloud_ptr, centroid);
    viewer->setCameraPosition(centroid[0]-140.0, centroid[1], centroid[2]+120.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);

    char cur_pc_id[10];
    sprintf(cur_pc_id, "pointcloud-%d", idx);
    std::string cur_pc_name(cur_pc_id);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(cur_transformed_cloud_ptr, "intensity");
    viewer->updatePointCloud<pcl::PointXYZI> (cur_transformed_cloud_ptr, intensity_distribution, cur_pc_name);

    Eigen::Matrix4d pre_pose = getGraphPose(pendingGraphOptimizer, pre_vex_id);
    Eigen::Matrix4d cur_pose = getGraphPose(pendingGraphOptimizer, idx);
    pcl::PointXYZ p1 = pcl::PointXYZ(pre_pose.cast<float>()(0,3), pre_pose.cast<float>()(1,3), pre_pose.cast<float>()(2,3));
    pcl::PointXYZ p2 = pcl::PointXYZ(cur_pose.cast<float>()(0,3), cur_pose.cast<float>()(1,3), cur_pose.cast<float>()(2,3));

    float line_color_factor = 1.0;//matchingScore > 400.0 ? 1.0 : (matchingScore < 100.0 ? 0.0 : (matchingScore-100.0)/400.0);
    std::cout << "line_color_factor: " << line_color_factor << std::endl;
    char line_id[10];
    sprintf(line_id, "new_line_%d-%d", pre_vex_id, idx);
    std::string line_name(line_id);
    viewer->addLine (p1, p2, 1.0-line_color_factor, line_color_factor, 0.0, line_name);

    std::vector<uint> loopclosing_id_vec;
    getLoopClosingPoseIds(pendingGraphOptimizer, idx, loopclosing_id_vec);
    for (uint v_id = 0; v_id < loopclosing_id_vec.size(); v_id++)
    {
      Eigen::Matrix4d closing_pose = getGraphPose(pendingGraphOptimizer, loopclosing_id_vec[v_id]);
      pcl::PointXYZ p3 = pcl::PointXYZ(closing_pose.cast<float>()(0,3), closing_pose.cast<float>()(1,3), closing_pose.cast<float>()(2,3));

      char closing_line_id[10];
      sprintf(closing_line_id, "new_line_%d-%d", loopclosing_id_vec[v_id], idx);
      std::cout << closing_line_id << std::endl;
      std::string closing_line_name(closing_line_id);
      viewer->addLine (p1, p3, 1.0-line_color_factor, line_color_factor, 0.0, closing_line_name);
    }

    if (_pause_to_debug)
      viewer->spin();
    else
      viewer->spinOnce(800);

    pre_cloud_ptr->swap(*cur_cloud_ptr);
    pre_vex_id = idx;
    printf(" ++++ \n");

  }

  viewer->spin();
}

void
  align_and_save_one_by_one_ndt (std::vector<std::string>& files)
{
  pcl::PointCloud<PointTypeIO>::Ptr pre_cloud_ptr(new pcl::PointCloud<PointTypeIO>);
  std::string extension (".pcd");
  std::string fname = files[0], ofname;
  if (fname.size () <= extension.size ())
  {
    std::cerr << "file name too short! " << std::endl;
    return;
  }
  //std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
  if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
  {
    // Load the cloud and saves it into the global list of models
    pcl::io::loadPCDFile (fname, *pre_cloud_ptr);
    ofname = fname;
    ofname.insert(ofname.size()-4,"_ndt");
  }

  pcl::PointCloud<PointTypeIO>::Ptr pre_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeIO>::Ptr pre_ground_ptr (new pcl::PointCloud<PointTypeIO>);
  remove_ground(pre_cloud_ptr, pre_no_ground_ptr, pre_ground_ptr);

  std::vector<pcl::PointCloud<PointTypeFull>::Ptr> segmented_component_ptrs;
  pcl::PointCloud<PointTypeFull>::Ptr pre_useful_cloud_ptr (new pcl::PointCloud<PointTypeFull>);
  pcl::PointCloud<PointTypeFull>::Ptr segmented_cloud_to_show_ptr (new pcl::PointCloud<PointTypeFull>);
  segment_cloud(pre_no_ground_ptr, segmented_component_ptrs, pre_useful_cloud_ptr, segmented_cloud_to_show_ptr);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  pcl::PointCloud<pcl::PointXYZ>::Ptr null_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  if (_enable_visualization)
  {
    viewer->setBackgroundColor (0, 0, 0);
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
    //viewer->addPointCloud<pcl::PointXYZI> (whole_cloud_out, intensity_distribution, "whole cloud");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");
    //viewer->addCoordinateSystem (1.0);
    //viewer->initCameraParameters ();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      target_color (null_cloud_ptr, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      output_color (null_cloud_ptr, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, output_color, "output cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "output cloud");

    // Coloring and visualizing initial transformed input cloud (gray).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      temp_color (null_cloud_ptr, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ> (null_cloud_ptr, temp_color, "temp cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1, "temp cloud");
  }

  Eigen::Matrix3f intrinsic (Eigen::Matrix3f::Identity ());
  Eigen::Matrix4f extrinsic (Eigen::Matrix4f::Identity ());
  Eigen::Matrix4d t (Eigen::Matrix4d::Identity ());
  Eigen::Matrix4d t_total (Eigen::Matrix4d::Identity ());

  g2o::VertexSE3* v = new g2o::VertexSE3;
  v->setId(0);
  // Eigen::AngleAxisd rotz(0.5, Eigen::Vector3d::UnitZ());
  // Eigen::AngleAxisd roty(-0.5, Eigen::Vector3d::UnitY());
  // Eigen::Matrix3d rot = (rotz * roty).toRotationMatrix();
  Eigen::Isometry3d iso3;
  iso3 = t_total.block<3,3>(0,0).cast<double>();
  iso3.translation() = t_total.block<3,1>(0,3).cast<double>();
  // iso3 = rot;
  // iso3.translation() = iso3.linear() * Eigen::Vector3d(3, 0, 0);
  v->setEstimate(iso3);
  vertices.push_back(v);
  std::cout << "vertex 0: " << std::endl;
  v->write(std::cout);
  std::cout << std::endl;

  //Eigen::Matrix<double,4,1> centroid;
  //pcl::compute3DCentroid(*pre_Cloud, centroid);
  //Eigen::AngleAxisf init_rotation (0.5, Eigen::Vector3f::UnitY ());
  //Eigen::Translation3f init_translation (centroid[0]-2.0, centroid[1], centroid[2]+2.0);
  //extrinsic = (init_translation * init_rotation).matrix ();

  //while (!viewer->wasStopped ())
  //{
  //  viewer->spinOnce (100);
  //  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  //}

  // int tmp_counter = 0;
  for (uint idx = 1; idx < files.size(); idx++)
  {
    pcl::PointCloud<PointTypeIO>::Ptr tmp_cloud_ptr(new pcl::PointCloud<PointTypeIO>);



    std::string fname = files[idx];
    // std::cout << "debug 1 " << fname << std::endl;
    if (fname.size () <= extension.size ())
      continue;
    //std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      pcl::io::loadPCDFile (fname, *tmp_cloud_ptr);
      ofname = fname;
      ofname.insert(ofname.size()-4,"_ndt");
    }
    // std::cout << "debug 2 " <<tmp_cloud_ptr->size() << std::endl;
    pcl::PointCloud<PointTypeIO>::Ptr tmp_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<PointTypeIO>::Ptr tmp_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    remove_ground(tmp_cloud_ptr, tmp_no_ground_ptr, tmp_ground_ptr);

    //std::vector<pcl::PointCloud<PointTypeFull>::Ptr> segmented_component_ptrs;
    pcl::PointCloud<PointTypeFull>::Ptr tmp_useful_cloud_ptr (new pcl::PointCloud<PointTypeFull>);
    //pcl::PointCloud<PointTypeFull>::Ptr segmented_cloud_to_show_ptr (new pcl::PointCloud<PointTypeFull>);
    segment_cloud(tmp_no_ground_ptr, segmented_component_ptrs, tmp_useful_cloud_ptr, segmented_cloud_to_show_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::copyPointCloud(*tmp_no_ground_ptr/*tmp_useful_cloud_ptr*/,*input_cloud_ptr);
    // pcl::copyPointCloud(*pre_no_ground_ptr/*pre_useful_cloud_ptr*/,*target_cloud_ptr);

    pcl::copyPointCloud(*tmp_no_ground_ptr/*+*tmp_ground_ptr*/, *input_cloud_ptr);
    pcl::copyPointCloud(*pre_no_ground_ptr/*+*pre_ground_ptr*/, *target_cloud_ptr);

    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setMaximumIterations (50);
    // icp.setMaxCorrespondenceDistance (5.0);
    // icp.setInputTarget (target_cloud_ptr);
    // icp.setInputCloud (input_cloud_ptr);
    // icp.align (*intermediate_cloud_ptr/*, t*/);
    // t = icp.getFinalTransformation ();

    pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp_nl;
    boost::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> > warp_fcn
      (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);
    boost::shared_ptr<pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> > te
      (new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>);
    te->setWarpFunction (warp_fcn);
    icp_nl.setTransformationEstimation (te);
    icp_nl.setMaximumIterations (50);
    icp_nl.setMaxCorrespondenceDistance (5.0);
    icp_nl.setRANSACOutlierRejectionThreshold (0.5);
    icp_nl.setInputTarget (target_cloud_ptr);
    icp_nl.setInputSource (input_cloud_ptr);
    icp_nl.align (*output_cloud_ptr/*, t*/);
    t = icp_nl.getFinalTransformation ().cast<double>();



    // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    // //ndt.setTransformationEpsilon (5.0);
    // //ndt.setStepSize (5.0);
    // //ndt.setResolution (50.0);
    // //ndt.setMaxCorrespondenceDistance(20.0);
    // //ndt.setMaximumIterations (50);
    //
    // //ndt.setInputSource (input_cloud_ptr);
    // //ndt.setInputTarget (target_cloud_ptr);
    // //ndt.align (*intermediate_cloud_ptr);
    // //t = ndt.getFinalTransformation();
    //
    // pcl::copyPointCloud(*tmp_no_ground_ptr/*+*tmp_ground_ptr*/, *input_cloud_ptr);
    // pcl::copyPointCloud(*pre_no_ground_ptr/*+*pre_ground_ptr*/, *target_cloud_ptr);
    //
    // ndt.setTransformationEpsilon (0.1);
    // ndt.setStepSize (0.1);
    // ndt.setResolution (0.5);
    // ndt.setMaxCorrespondenceDistance(3.0);
    // ndt.setMaximumIterations (50);
    //
    // ndt.setInputSource (input_cloud_ptr);
    // ndt.setInputTarget (target_cloud_ptr);
    // ndt.align (*output_cloud_ptr, t);
    // t = ndt.getFinalTransformation();

    std::cout << "\n Scan-matching has converged:" << icp_nl.hasConverged ()
      << " score: " << icp_nl.getFitnessScore () << std::endl;

    std::cout << "\n" << "Transformation Matrix: \n" << t/*ndt.getFinalTransformation()*/ << std::endl;

    if (_enable_visualization)
    {
      Eigen::Matrix<double,4,1> centroid;
      pcl::compute3DCentroid(*pre_cloud_ptr, centroid);
      Eigen::AngleAxisf init_rotation (1.0, Eigen::Vector3f::UnitY ());
      Eigen::Translation3f init_translation (centroid[0]-15.0, centroid[1], centroid[2]+15.0);
      extrinsic = (init_translation * init_rotation).matrix ();
      //viewer->setCameraParameters(intrinsic, extrinsic);
      viewer->setCameraPosition(centroid[0]-140.0, centroid[1], centroid[2]+120.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);
      //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
      //viewer->updatePointCloud<pcl::PointXYZI>(whole_cloud_out, intensity_distribution, "whole cloud");
      //viewer->removeAllPointClouds();
      //viewer->addPointCloud(wholeCloud);

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_color (target_cloud_ptr, 255, 0, 0);
      viewer->updatePointCloud<pcl::PointXYZ> (target_cloud_ptr, target_color, "target cloud");
      //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      //  1, "target cloud");

      // Coloring and visualizing transformed input cloud (green).
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        output_color (output_cloud_ptr, 0, 255, 0);
      viewer->updatePointCloud<pcl::PointXYZ> (output_cloud_ptr, output_color, "output cloud");
      //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      //  1, "output cloud");

      // Coloring and visualizing initial transformed input cloud (gray).
      //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      //  temp_color (intermediate_cloud_ptr, 0, 0, 255);
      //viewer->updatePointCloud<pcl::PointXYZ> (intermediate_cloud_ptr, temp_color, "temp cloud");
      //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      //  1, "temp cloud");

      if (_pause_to_debug)
        viewer->spin();
      else
        viewer->spinOnce();
    }

    t_total = t_total * t;
    pcl::PointCloud<PointTypeIO>::Ptr global_output_cloud_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::transformPointCloud (*tmp_cloud_ptr/*tmp_no_ground_ptr*/, *global_output_cloud_ptr, t_total);

    pre_cloud_ptr->swap(*tmp_cloud_ptr);
    pre_no_ground_ptr->swap(*tmp_no_ground_ptr);
    pre_ground_ptr->swap(*tmp_ground_ptr);
    printf(" ++++ \n");

    g2o::VertexSE3* v = new g2o::VertexSE3;
    v->setId(idx);
    Eigen::Isometry3d iso3;
    iso3 = t_total.block<3,3>(0,0).cast<double>();
    iso3.translation() = t_total.block<3,1>(0,3).cast<double>();
    v->setEstimate(iso3);
    vertices.push_back(v);
    std::cout << "[vertex " << idx << "] ";
    v->write(std::cout);
    std::cout << std::endl;

    Eigen::Matrix3d transNoise = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i)
      transNoise(i, i) = std::pow(icp_nl.getFitnessScore (), 0.5);
    std::cout << "translation noise: " << std::pow(transNoise(0, 0),0.5) << std::endl;

    Eigen::Matrix3d rotNoise = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i)
      rotNoise(i, i) = std::pow(icp_nl.getFitnessScore (), 0.5) * 0.001745;
    std::cout << "rotation noise: " << std::pow(rotNoise(0, 0),0.5) << std::endl;

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    information.block<3,3>(0,0) = transNoise.inverse();
    information.block<3,3>(3,3) = rotNoise.inverse();

    g2o::VertexSE3* prev = vertices[idx-1];
    g2o::VertexSE3* cur  = vertices[idx];
    Eigen::Isometry3d iso3_edge = prev->estimate().inverse() * cur->estimate();
    g2o::EdgeSE3* e = new g2o::EdgeSE3;
    e->setVertex(0, prev);
    e->setVertex(1, cur);
    e->setMeasurement(iso3_edge);
    e->setInformation(information);
    odometryEdges.push_back(e);
    edges.push_back(e);

    std::cout << "[edge " << idx-1 << "-" << idx << "] ";
    e->write(std::cout);
    std::cout << endl;

    std::cout << "output point cloud: " << ofname << std::endl;
    // pcl::io::savePCDFileASCII (ofname, *global_output_cloud_ptr);

    //  wholeCloud->clear();
    //}

  }
}

void
  align_one_by_one_ndt (std::vector<pcl::PointCloud<PointTypeIO>::Ptr>& clouds_in,
                        const pcl::PointCloud<PointTypeIO>::Ptr &whole_cloud_out)
{

  pcl::PointCloud<PointTypeIO>::Ptr pre_cloud_ptr = *(clouds_in.begin());

  pcl::PointCloud<PointTypeIO>::Ptr pre_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeIO>::Ptr pre_ground_ptr (new pcl::PointCloud<PointTypeIO>);
  remove_ground(pre_cloud_ptr, pre_no_ground_ptr, pre_ground_ptr);
  //pcl::PassThrough<PointTypeIO> pass_xyzi;
  //pass_xyzi.setInputCloud (pre_cloud_ptr);
  //pass_xyzi.setFilterFieldName ("z");
  //pass_xyzi.setFilterLimits (-1.5, 100);
  ////pass.setFilterLimitsNegative (true);
  //pass_xyzi.filter (*pre_cloud_ptr);

  pcl::copyPointCloud(*pre_cloud_ptr, *whole_cloud_out);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  if (_enable_visualization){
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
    viewer->addPointCloud<pcl::PointXYZI> (whole_cloud_out, intensity_distribution, "whole cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
  }

  Eigen::Matrix3f intrinsic (Eigen::Matrix3f::Identity ());
  Eigen::Matrix4f extrinsic (Eigen::Matrix4f::Identity ());
  Eigen::Matrix4d t (Eigen::Matrix4d::Identity ());

  //Eigen::Matrix<double,4,1> centroid;
  //pcl::compute3DCentroid(*pre_Cloud, centroid);
  //Eigen::AngleAxisf init_rotation (0.5, Eigen::Vector3f::UnitY ());
  //Eigen::Translation3f init_translation (centroid[0]-2.0, centroid[1], centroid[2]+2.0);
  //extrinsic = (init_translation * init_rotation).matrix ();

  //while (!viewer->wasStopped ())
  //{
  //  viewer->spinOnce (100);
  //  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  //}

  // int tmp_counter = 0;
  for (std::vector<pcl::PointCloud<PointTypeIO>::Ptr>::iterator clouds_iter = clouds_in.begin()+1; clouds_iter!=clouds_in.end(); clouds_iter++)
  {
    pcl::PointCloud<PointTypeIO>::Ptr tmp_cloud_ptr = *clouds_iter;

    pcl::PointCloud<PointTypeIO>::Ptr tmp_no_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<PointTypeIO>::Ptr tmp_ground_ptr (new pcl::PointCloud<PointTypeIO>);
    remove_ground(tmp_cloud_ptr, tmp_no_ground_ptr, tmp_ground_ptr);
    ////pcl::PassThrough<pcl::PointXYZ> pass;
    //pass_xyzi.setInputCloud (tmp_cloud_ptr);
    //pass_xyzi.setFilterFieldName ("z");
    //pass_xyzi.setFilterLimits (-1.5, 100);
    ////pass.setFilterLimitsNegative (true);
    //pass_xyzi.filter (*tmp_cloud_ptr);

    //Eigen::Matrix<double,4,1> centroid;
    //pcl::compute3DCentroid(*tmp_cloud, centroid);
    //std::cout << "centroid: " << centroid[0] << " " << centroid[1] << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pre_no_ground_ptr,*input_cloud_ptr);
    pcl::copyPointCloud(*tmp_no_ground_ptr,*target_cloud_ptr);
    //std::vector<double> source_z_values; source_z_values.reserve(input_cloud->size());
    //std::vector<double> target_z_values; target_z_values.reserve(target_cloud->size());


    //for (unsigned int i = 0; i<input_cloud->size(); i++){
    //  source_z_values[i] = input_cloud->points[i].z;
    //  input_cloud->points[i].z = 0;
    //}

    /*
    pcl::NormalDistributionsTransform2D<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.3);
    ndt.setGridCentre(Eigen::Vector2f(centroid[0],centroid[1]));
    ndt.setGridStep(Eigen::Vector2f(0.2,0.2));
    ndt.setGridExtent(Eigen::Vector2f(100.0,100.0));
    ndt.setMaxCorrespondenceDistance(0.3);
    ndt.setMaximumIterations(35);

    ndt.setInputSource(input_cloud);
    ndt.setInputTarget(target_cloud);

    ndt.align(*output_cloud);
    */
    //pcl::PassThrough<pcl::PointXYZ> pass;
    //pass.setInputCloud (input_cloud_ptr);
    //pass.setFilterFieldName ("z");
    //pass.setFilterLimits (-1.5, 100);
    ////pass.setFilterLimitsNegative (true);
    //pass.filter (*input_cloud_ptr);

    //pass.setInputCloud (target_cloud_ptr);
    //pass.setFilterFieldName ("z");
    //pass.setFilterLimits (-1.5, 100);
    ////pass.setFilterLimitsNegative (true);
    //pass.filter (*target_cloud_ptr);

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon (0.1);
    ndt.setStepSize (0.1);
    ndt.setResolution (1.0);
    ndt.setMaxCorrespondenceDistance(3.0);
    ndt.setMaximumIterations (35);
    ndt.setInputSource (input_cloud_ptr);
    ndt.setInputTarget (target_cloud_ptr);

    ndt.align (*output_cloud_ptr);

    t = ndt.getFinalTransformation().cast<double>();

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
      << " score: " << ndt.getFitnessScore () << std::endl;

    pcl::PointCloud<PointTypeIO>::Ptr tmp_whole_cloud_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::transformPointCloud (*whole_cloud_out, *tmp_whole_cloud_ptr, t);
    /*
    Eigen::AngleAxisf init_rotation (d_theta, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (d_x, d_y, d_z);
    Eigen::Matrix4d init_guess = (init_translation * init_rotation).matrix ();
    pcl::transformPointCloud (*input_cloud, *temp_cloud, init_guess);
    */
    //for (unsigned int i = 0; i<output_cloud->size(); i++){
    //  output_cloud->points[i].z = source_z_values[i];
    //}

    std::cout << "\n" << "Transformation Matrix: \n" << t/*ndt.getFinalTransformation()*/ << std::endl;

    *whole_cloud_out = *tmp_whole_cloud_ptr + *tmp_cloud_ptr;
    //*pre_cloud_ptr = *tmp_cloud_ptr;
    pre_cloud_ptr->swap(*tmp_cloud_ptr);
    pre_no_ground_ptr->swap(*tmp_no_ground_ptr);
    printf(" ++++ ");

    //tmp_counter++;
    //std::cout << "counter value: " << tmp_counter%10 << std::endl;
    //if (tmp_counter%10==0){
    //  std::cout << std::endl << "coming..#############################.." << std::endl;
    if (_enable_visualization){
      Eigen::Matrix<double,4,1> centroid;
      pcl::compute3DCentroid(*pre_cloud_ptr, centroid);
      Eigen::AngleAxisf init_rotation (0.5, Eigen::Vector3f::UnitY ());
      Eigen::Translation3f init_translation (centroid[0]-2.0, centroid[1], centroid[2]+2.0);
      extrinsic = (init_translation * init_rotation).matrix ();
      //viewer->setCameraParameters(intrinsic, extrinsic);
      viewer->setCameraPosition(centroid[0]-40.0, centroid[1], centroid[2]+20.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
      viewer->updatePointCloud<pcl::PointXYZI>(whole_cloud_out, intensity_distribution, "whole cloud");
      //viewer->removeAllPointClouds();
      //viewer->addPointCloud(wholeCloud);
      viewer->spinOnce(1000);
    }
    //  wholeCloud->clear();
    //}

  }
}

void
  align_one_by_one (std::vector<pcl::PointCloud<PointTypeIO>::Ptr>& clouds_in,
                    const pcl::PointCloud<PointTypeIO>::Ptr &whole_cloud_out)
{

  pcl::PointCloud<PointTypeIO>::Ptr pre_cloud_ptr = *(clouds_in.begin());

  pcl::PassThrough<PointTypeIO> pass_xyzi;
  pass_xyzi.setInputCloud (pre_cloud_ptr);
  pass_xyzi.setFilterFieldName ("z");
  pass_xyzi.setFilterLimits (-1.7, 100);
  //pass.setFilterLimitsNegative (true);
  pass_xyzi.filter (*pre_cloud_ptr);


  pcl::copyPointCloud(*pre_cloud_ptr, *whole_cloud_out);
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(wholeCloud, "intensity");
  //viewer->addPointCloud<pcl::PointXYZI> (wholeCloud, intensity_distribution, "whole cloud");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");
  //viewer->addCoordinateSystem (1.0);
  //viewer->initCameraParameters ();
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  if (_enable_visualization){
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
    viewer->addPointCloud<pcl::PointXYZI> (whole_cloud_out, intensity_distribution, "whole cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
  }



  Eigen::Matrix3f intrinsic (Eigen::Matrix3f::Identity ());
  Eigen::Matrix4f extrinsic (Eigen::Matrix4f::Identity ());
  Eigen::Matrix4d t (Eigen::Matrix4d::Identity ());

  //Eigen::Matrix<double,4,1> centroid;
  //pcl::compute3DCentroid(*pre_Cloud, centroid);
  //Eigen::AngleAxisf init_rotation (0.5, Eigen::Vector3f::UnitY ());
  //Eigen::Translation3f init_translation (centroid[0]-2.0, centroid[1], centroid[2]+2.0);
  //extrinsic = (init_translation * init_rotation).matrix ();

  //while (!viewer->wasStopped ())
  //{
  //  viewer->spinOnce (100);
  //  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  //}

  // int tmp_counter = 0;
  for (std::vector<pcl::PointCloud<PointTypeIO>::Ptr>::iterator clouds_iter = clouds_in.begin()+1; clouds_iter!=clouds_in.end(); clouds_iter++)
  {
    pcl::PointCloud<PointTypeIO>::Ptr tmp_cloud_ptr = *clouds_iter;
    //pcl::PassThrough<pcl::PointXYZ> pass;
    pass_xyzi.setInputCloud (tmp_cloud_ptr);
    pass_xyzi.setFilterFieldName ("z");
    pass_xyzi.setFilterLimits (-1.7, 100);
    //pass.setFilterLimitsNegative (true);
    pass_xyzi.filter (*tmp_cloud_ptr);

    //Eigen::Matrix<double,4,1> centroid;
    //pcl::compute3DCentroid(*tmp_cloud, centroid);
    //std::cout << "centroid: " << centroid[0] << " " << centroid[1] << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pre_cloud_ptr,*input_cloud_ptr);
    pcl::copyPointCloud(*tmp_cloud_ptr,*target_cloud_ptr);

    //std::vector<double> source_z_values; source_z_values.reserve(input_cloud->size());
    //std::vector<double> target_z_values; target_z_values.reserve(target_cloud->size());


    //for (unsigned int i = 0; i<input_cloud->size(); i++){
    //  source_z_values[i] = input_cloud->points[i].z;
    //  input_cloud->points[i].z = 0;
    //}

    /*
    pcl::NormalDistributionsTransform2D<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.3);
    ndt.setGridCentre(Eigen::Vector2f(centroid[0],centroid[1]));
    ndt.setGridStep(Eigen::Vector2f(0.2,0.2));
    ndt.setGridExtent(Eigen::Vector2f(100.0,100.0));
    ndt.setMaxCorrespondenceDistance(0.3);
    ndt.setMaximumIterations(35);

    ndt.setInputSource(input_cloud);
    ndt.setInputTarget(target_cloud);

    ndt.align(*output_cloud);
    */

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (input_cloud_ptr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1.7, 100);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*input_cloud_ptr);

    pass.setInputCloud (target_cloud_ptr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1.7, 100);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*target_cloud_ptr);

    pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;

    boost::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> > warp_fcn
      (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);

    // Create a TransformationEstimationLM object, and set the warp to it
    boost::shared_ptr<pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> > te
      (new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>);
    te->setWarpFunction (warp_fcn);

    // Pass the TransformationEstimation objec to the ICP algorithm
    icp.setTransformationEstimation (te);

    icp.setMaximumIterations (50);
    icp.setMaxCorrespondenceDistance (0.5);
    icp.setRANSACOutlierRejectionThreshold (0.5);

    icp.setInputTarget (target_cloud_ptr);
    icp.setInputSource (input_cloud_ptr);

    icp.align (*output_cloud_ptr);

    t = icp.getFinalTransformation ().cast<double>() /** t*/;
    //extrinsic = t * extrinsic;

    pcl::PointCloud<PointTypeIO>::Ptr tmp_whole_cloud_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::transformPointCloud<PointTypeIO> (*whole_cloud_out, *tmp_whole_cloud_ptr, t);

    // std::cout << icp.getFinalTransformation () << std::endl;

    //for (unsigned int i = 0; i<output_cloud->size(); i++){
    //  output_cloud->points[i].z = source_z_values[i];
    //}

    std::cout << "\n" << "Transformation Matrix: \n" << t/*ndt.getFinalTransformation()*/ << std::endl;

    //pcl::concatenateFields(*lastCloud,*output_cloud,*lastCloud);

    *whole_cloud_out = *tmp_whole_cloud_ptr + *tmp_cloud_ptr;
    *pre_cloud_ptr = *tmp_cloud_ptr;
    printf(" ++++ ");

    if (_enable_visualization){
      Eigen::Matrix<double,4,1> centroid;
      pcl::compute3DCentroid(*pre_cloud_ptr, centroid);
      Eigen::AngleAxisf init_rotation (0.5, Eigen::Vector3f::UnitY ());
      Eigen::Translation3f init_translation (centroid[0]-2.0, centroid[1], centroid[2]+2.0);
      extrinsic = (init_translation * init_rotation).matrix ();
      //viewer->setCameraParameters(intrinsic, extrinsic);
      viewer->setCameraPosition(centroid[0]-40.0, centroid[1], centroid[2]+20.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_out, "intensity");
      viewer->updatePointCloud<pcl::PointXYZI>(whole_cloud_out, intensity_distribution, "whole cloud");
      //viewer->removeAllPointClouds();
      //viewer->addPointCloud(wholeCloud);
      viewer->spinOnce(1000);
    }

    //tmp_counter++;
    //std::cout << "counter value: " << tmp_counter%10 << std::endl;
    //if (tmp_counter%10==0){
    //  std::cout << std::endl << "coming..#############################.." << std::endl;
    //  Eigen::Matrix<double,4,1> centroid;
    //  pcl::compute3DCentroid(*pre_Cloud, centroid);
    //  Eigen::AngleAxisf init_rotation (0.5, Eigen::Vector3f::UnitY ());
    //  Eigen::Translation3f init_translation (centroid[0]-2.0, centroid[1], centroid[2]+2.0);
    //  extrinsic = (init_translation * init_rotation).matrix ();
    //  //viewer->setCameraParameters(intrinsic, extrinsic);
    //  viewer->setCameraPosition(centroid[0]-40.0, centroid[1], centroid[2]+20.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);
    //  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(wholeCloud, "intensity");
    //  viewer->updatePointCloud<pcl::PointXYZI>(wholeCloud, intensity_distribution, "whole cloud");
    //  //viewer->removeAllPointClouds();
    //  //viewer->addPointCloud(wholeCloud);
    //  viewer->spinOnce(1000);
    //  wholeCloud->clear();
    //}

  }
}

void
  align_one_by_one_2d (std::vector<pcl::PointCloud<PointTypeIO>::Ptr>& clouds_in,
                       const pcl::PointCloud<PointTypeIO>::Ptr &whole_cloud_out_ptr)
{
  for (std::vector<pcl::PointCloud<PointTypeIO>::Ptr>::iterator clouds_iter = clouds_in.begin()+1; clouds_iter!=clouds_in.end(); clouds_iter++)
  {
    pcl::PointCloud<pcl::PointXY>::Ptr xy_input_cloud_ptr (new pcl::PointCloud<pcl::PointXY>);
    pcl::PointCloud<pcl::PointXY>::Ptr xy_target_cloud_ptr (new pcl::PointCloud<pcl::PointXY>);

    pcl::copyPointCloud(**(clouds_iter-1), *xy_input_cloud_ptr);
    pcl::copyPointCloud(**clouds_iter, *xy_target_cloud_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*xy_input_cloud_ptr, *input_cloud_ptr);
    pcl::copyPointCloud(*xy_target_cloud_ptr, *target_cloud_ptr);

    pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;

    boost::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> > warp_fcn
      (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);
    // Create a TransformationEstimationLM object, and set the warp to it
    boost::shared_ptr<pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> > te
      (new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>);

    te->setWarpFunction (warp_fcn);
    // Pass the TransformationEstimation object to the ICP algorithm
    icp.setTransformationEstimation (te);

    icp.setMaximumIterations (50);
    icp.setMaxCorrespondenceDistance (0.5);
    icp.setRANSACOutlierRejectionThreshold (0.5);

    icp.setInputTarget (target_cloud_ptr);
    icp.setInputSource (input_cloud_ptr);

    icp.align (*output_cloud_ptr);

    Eigen::Matrix4d t = icp.getFinalTransformation ().cast<double>();

    pcl::PointCloud<PointTypeIO>::Ptr tmp_whole_cloud_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::transformPointCloud<PointTypeIO> (*whole_cloud_out_ptr, *tmp_whole_cloud_ptr, t);

    std::cout << "\n" << "Transformation Matrix: \n" << t << std::endl;


    *whole_cloud_out_ptr = *tmp_whole_cloud_ptr + **clouds_iter;
    printf(" ++++ ");

  }
}

void
  align_each_one (std::vector<pcl::PointCloud<PointTypeIO>::Ptr>& clouds_in,
                  std::vector<pcl::PointCloud<PointTypeIO>::Ptr>& clouds_out,
                  const pcl::PointCloud<PointTypeIO>::Ptr &whole_cloud_out)
{
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(wholeCloud, "intensity");
  //viewer->addPointCloud<pcl::PointXYZI> (wholeCloud, intensity_distribution, "whole cloud");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");
  //viewer->addCoordinateSystem (1.0);
  //viewer->initCameraParameters ();
  //pcl::PointCloud<PointTypeIO>::Ptr pre_cloud_ptr = *(clouds_in.begin());
  Eigen::Matrix3f intrinsic (Eigen::Matrix3f::Identity ());
  Eigen::Matrix4f extrinsic (Eigen::Matrix4f::Identity ());
  Eigen::Matrix4d t (Eigen::Matrix4d::Identity ());

  pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;

  boost::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> > warp_fcn
    (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);
  // Create a TransformationEstimationLM object, and set the warp to it
  boost::shared_ptr<pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> > te
    (new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>);

  te->setWarpFunction (warp_fcn);
  // Pass the TransformationEstimation object to the ICP algorithm
  icp.setTransformationEstimation (te);

  icp.setMaximumIterations (50);
  icp.setMaxCorrespondenceDistance (0.5);
  icp.setRANSACOutlierRejectionThreshold (0.5);

  //Eigen::Matrix<double,4,1> centroid;
  //pcl::compute3DCentroid(*pre_Cloud, centroid);
  //Eigen::AngleAxisf init_rotation (0.5, Eigen::Vector3f::UnitY ());
  //Eigen::Translation3f init_translation (centroid[0]-2.0, centroid[1], centroid[2]+2.0);
  //extrinsic = (init_translation * init_rotation).matrix ();

  //while (!viewer->wasStopped ())
  //{
  //  viewer->spinOnce (100);
  //  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  //}

  for (std::vector<pcl::PointCloud<PointTypeIO>::Ptr>::iterator clouds_iter = clouds_in.begin(); clouds_iter!=clouds_in.end(); clouds_iter++)
  {
    //pcl::PointCloud<PointTypeIO>::Ptr tmp_input_cloud_ptr = *clouds_iter;
    //pcl::PointCloud<PointTypeIO>::Ptr tmp_target_cloud_ptr (new pcl::PointCloud<PointTypeIO>);
    //*tmp_target_cloud_ptr

    //Eigen::Matrix<double,4,1> centroid;
    //pcl::compute3DCentroid(*tmp_cloud, centroid);
    //std::cout << "centroid: " << centroid[0] << " " << centroid[1] << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(**clouds_iter,*input_cloud_ptr);

    if ( clouds_iter == clouds_in.begin() )
    {
      pcl::copyPointCloud(**(clouds_iter+1), *target_cloud_ptr);
    }
    else if ( clouds_iter == clouds_in.end() - 1 )
    {
      pcl::copyPointCloud(**(clouds_iter-1), *target_cloud_ptr);
    }
    else
    {
      pcl::copyPointCloud(**(clouds_iter-1)+**(clouds_iter+1), *target_cloud_ptr );
    }

    icp.setInputTarget (target_cloud_ptr);
    icp.setInputSource (input_cloud_ptr);

    icp.align (*output_cloud_ptr);
    t = icp.getFinalTransformation ().cast<double>();

    pcl::PointCloud<PointTypeIO>::Ptr tmp_cloud_ptr (new pcl::PointCloud<PointTypeIO>);
    pcl::transformPointCloud<PointTypeIO> (**clouds_iter, *tmp_cloud_ptr, t);
    clouds_out.emplace_back(tmp_cloud_ptr);

    std::cout << "\n" << "Transformation Matrix: \n" << t << std::endl;

    *whole_cloud_out = *whole_cloud_out + *tmp_cloud_ptr;
    printf(" ++++ ");

  }
}

void
  handlingSGMPointCloud(std::vector<std::string>& files)
{
  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////                Creating SGMs                  /////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  boost::shared_ptr<pcl::visualization::PCLVisualizer> temp_viewer (new pcl::visualization::PCLVisualizer ("temp Viewer"));
  pcl::PointCloud<pcl::PointXYZI>::Ptr null_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  null_handler(null_cloud_ptr, "intensity");
  temp_viewer->addPointCloud<pcl::PointXYZI> (null_cloud_ptr, null_handler, "temp cloud");

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

//  parseCommandLine(argc, argv);
//  std::cout << "commands parsed." << std::endl;

  // for (int i=0; i<_initial_poses.size();i=i+10)
  // {
  //   //std::cout << i <<" . ";
  //   //Eigen::Affine3f pse_pose = vertex_transformations[i].cast<float>();
  //   Eigen::Transform<float, 3, Eigen::Affine> pse_pose(vertex_transformations[i].cast<float>());
  //   //std::cout << pse_pose.translation()[0] << " " << pse_pose.translation()[1] << std::endl;
  //   Eigen::Quaternion<float> q(pse_pose.rotation());
  //   //std::cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  //   viewer->addCoordinateSystem (1.0, pse_pose);

  //   // char a[10];
  //   // sprintf(a, "%05d", i);
  //   // std::string ss("cube");
  //   // ss.append(a);
  // }

  pcl::ModelCoefficients coeffs;
  // translation
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  // rotation
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  // size
  coeffs.values.push_back (3.5);
  coeffs.values.push_back (1.8);
  coeffs.values.push_back (1.3);
  viewer->removeShape ("cube");
  viewer->addCube (coeffs, "cube");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,1.0,1.0,"cube");

//  printf("loading");
//  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
//  loadData (files, clouds);
//  printf("loaded");
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
  pcl::PointCloud<pcl::PointXYZI>::Ptr whole_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  std::cout << "To show clouds form id:" << _file_start_id << " to id:" << _file_end_id << " with step " << _file_id_step << std::endl;
  for ( uint idx = (_file_id_step > 0) ? (_file_start_id) : (_file_end_id);
        ( _file_id_step > 0 && idx <= _file_end_id ) || ( _file_id_step < 0 && idx >= _file_start_id );
        idx = idx + _file_id_step )
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    std::string fname = files[idx];
    // std::cout << "debug 1 " << fname << std::endl;

    std::string extension ("");
    if (fname.size () <= extension.size ())
      continue;
    //std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      pcl::io::loadPCDFile (fname, *temp_cloud_ptr);
    }

//    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
//    remove_ground(temp_cloud_ptr, tmp_no_ground_ptr, tmp_ground_ptr);

    // pcl::io::loadPCDFile (files[cloud_id], *temp_cloud_ptr);
    Eigen::Matrix4d initial_pose (Eigen::Matrix4d::Identity());
    if (HAVE_GRAPH_INPUT)
    {
      uint pre_query_vertex_id = idx - _file_id_step;
      uint cur_query_vertex_id = idx;
      initial_pose = getGraphPose(optimizerForLoadingInput, cur_query_vertex_id);
    }
    else if (HAVE_GPS_INPUT)
    {
      uint pre_query_pcd_file_idx = idx - _file_id_step;
      uint cur_query_pcd_file_idx = idx;
      initial_pose = getGPSPose(cur_query_pcd_file_idx);
    }
    else if (_remove_z_offset)
    {
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D (*temp_cloud_ptr, min_pt, max_pt);
      std::cout << "min_pt: " << min_pt << std::endl;
      std::cout << "max_pt: " << max_pt << std::endl;
      initial_pose(2,3) = -min_pt(2)-2.0;
    }
    pcl::transformPointCloud(*temp_cloud_ptr, *trans_cloud_ptr, initial_pose.cast<float>());
    std::cout << "\n" << "Transformation Matrix: \n" << initial_pose << std::endl;

    //Eigen::Affine3f pse_pose = vertex_transformations[i].cast<float>();
    Eigen::Transform<float, 3, Eigen::Affine> pse_pose(initial_pose.cast<float>());
    //std::cout << pse_pose.translation()[0] << " " << pse_pose.translation()[1] << std::endl;
    // Eigen::Quaternion<float> q(pse_pose.rotation());
    //std::cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    viewer->addCoordinateSystem (1.0, pse_pose);

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (trans_cloud_ptr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.0, 6.0);
    pass.setFilterLimitsNegative (false);
    pass.filter (*trans_cloud_ptr);

    clouds.emplace_back(trans_cloud_ptr);
    *whole_cloud_ptr += *trans_cloud_ptr;
    std::cout << files[idx] << " loaded. " << std::endl;

//    if (_visualize_individually)
//    {
//      Eigen::Matrix<double,4,1> temp_centroid;
//      pcl::compute3DCentroid(*trans_cloud_ptr, temp_centroid);
//      temp_viewer->setCameraPosition(temp_centroid[0]-140.0, temp_centroid[1], temp_centroid[2]+120.0, temp_centroid[0], temp_centroid[1], temp_centroid[2], 1, 0, 2);
//      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  color_handler(trans_cloud_ptr, "intensity");
//      temp_viewer->updatePointCloud<pcl::PointXYZI> (trans_cloud_ptr, color_handler, "temp cloud");
//
//      temp_viewer->spinOnce(200);
//    }
  }
  std::cout << " \n  total points number: " << whole_cloud_ptr->size() << std::endl;
//  SaveMarkings("markings.txt", clouds);
//  pcl::io::savePCDFileASCII ("whole_cloud.pcd", *whole_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
//  pcl::copyPointCloud(*wholeCloud,*filteredCloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr removedPoints(new pcl::PointCloud<pcl::PointXYZI>);
  //pcl::PointIndices::Ptr removedPointsIndicesPtr (new pcl::PointIndices);
  //pcl::PointIndices filteredPointsIndices;

  if (_do_filter)
  {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (whole_cloud_ptr);
    sor.setMeanK (param1);
    sor.setStddevMulThresh (param2);
    sor.setNegative(false);
    sor.filter (*filteredCloud);
    sor.setNegative(true);
    sor.filter (*removedPoints);
  }
  else
    pcl::copyPointCloud(*whole_cloud_ptr,*filteredCloud);

  std::vector<pcl::PointCloud<PointTypeFull>::Ptr> segmented_component_ptrs;
  pcl::PointCloud<PointTypeFull>::Ptr pre_useful_cloud_ptr (new pcl::PointCloud<PointTypeFull>);
  pcl::PointCloud<PointTypeFull>::Ptr segmented_cloud_to_show_ptr (new pcl::PointCloud<PointTypeFull>);
  segment_cloud(filteredCloud, segmented_component_ptrs, pre_useful_cloud_ptr, segmented_cloud_to_show_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr points_to_visualize(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*segmented_cloud_to_show_ptr,*points_to_visualize);

  pcl::PointCloud<pcl::PointXYZ>::Ptr plain_whole_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*filteredCloud,*plain_whole_cloud_ptr);

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(points_to_visualize, "intensity");
  viewer->addPointCloud<pcl::PointXYZI> (points_to_visualize, intensity_distribution, "whole cloud");

  if (!output_pcd_file_name.empty())
  {
    pcl::io::savePCDFileASCII (output_pcd_file_name.c_str(), *whole_cloud_ptr);
    std::cout << "pcd saved: " << output_pcd_file_name << std::endl;
  }

  if (!output_markings_file_name.empty())
  {
    if (!segmented_component_ptrs.empty())
      SaveMarkings(output_markings_file_name.c_str(), segmented_component_ptrs);
    else
      SaveMarkings(output_markings_file_name.c_str(), clouds);
    std::cout << "markings saved: " << output_markings_file_name << std::endl;
  }

  Eigen::Matrix<double,4,1> centroid;
  pcl::compute3DCentroid(*whole_cloud_ptr, centroid);
//  Eigen::AngleAxisf init_rotation (1.0, Eigen::Vector3f::UnitY ());
//  Eigen::Translation3f init_translation (centroid[0]-15.0, centroid[1], centroid[2]+15.0);
//  extrinsic = (init_translation * init_rotation).matrix ();
  //viewer->setCameraParameters(intrinsic, extrinsic);
  viewer->setCameraPosition(centroid[0]-140.0, centroid[1], centroid[2]+120.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);

//  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_ptr, "intensity");
//  viewer->addPointCloud<pcl::PointXYZI> (whole_cloud_ptr, intensity_distribution, "whole cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");


//  pcl::PointCloud<pcl::PointXYZI>::Ptr ibeo_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
//  pcl::io::loadPCDFile ("ibeo.pcd", *ibeo_cloud_ptr);
//  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  ibeo_intensity_distribution(ibeo_cloud_ptr, "intensity");
//  viewer->addPointCloud<pcl::PointXYZI> (ibeo_cloud_ptr, ibeo_intensity_distribution, "ibeo cloud");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ibeo cloud");


  viewer->spin();
  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////              end Creating SGMs                /////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
}

void
  view(std::vector<std::string>& files)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
  pcl::PointCloud<pcl::PointXYZI>::Ptr whole_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  std::cout << "To show clouds form id:" << _file_start_id << " to id:" << _file_end_id << " with step " << _file_id_step << std::endl;
  for ( uint idx = (_file_id_step > 0) ? (_file_start_id) : (_file_end_id);
        ( _file_id_step > 0 && idx <= _file_end_id ) || ( _file_id_step < 0 && idx >= _file_start_id );
        idx = idx + _file_id_step )
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    std::string fname = files[idx];
    // std::cout << "debug 1 " << fname << std::endl;

    std::string extension ("");
    if (fname.size () <= extension.size ())
      continue;
    //std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      pcl::io::loadPCDFile (fname, *temp_cloud_ptr);
    }

//    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
//    remove_ground(temp_cloud_ptr, tmp_no_ground_ptr, tmp_ground_ptr);

    // pcl::io::loadPCDFile (files[cloud_id], *temp_cloud_ptr);
    Eigen::Matrix4d initial_pose (Eigen::Matrix4d::Identity());
    if (HAVE_GRAPH_INPUT)
    {
      uint pre_query_vertex_id = idx - _file_id_step;
      uint cur_query_vertex_id = idx;
      initial_pose = getGraphPose(optimizerForLoadingInput, cur_query_vertex_id);
    }
    else if (HAVE_GPS_INPUT)
    {
      uint pre_query_pcd_file_idx = idx - _file_id_step;
      uint cur_query_pcd_file_idx = idx;
      initial_pose = getGPSPose(cur_query_pcd_file_idx);
    }
    pcl::transformPointCloud(*temp_cloud_ptr, *trans_cloud_ptr, initial_pose.cast<float>());
    std::cout << "\n" << "Transformation Matrix: \n" << initial_pose << std::endl;

    //Eigen::Affine3f pse_pose = vertex_transformations[i].cast<float>();
    Eigen::Transform<float, 3, Eigen::Affine> pse_pose(initial_pose.cast<float>());
    //std::cout << pse_pose.translation()[0] << " " << pse_pose.translation()[1] << std::endl;
    // Eigen::Quaternion<float> q(pse_pose.rotation());
    //std::cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    viewer->addCoordinateSystem (1.0, pse_pose);

//    pcl::PassThrough<pcl::PointXYZI> pass;
//    pass.setInputCloud (trans_cloud_ptr);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (-0.0, 6.0);
//    pass.setFilterLimitsNegative (false);
//    pass.filter (*trans_cloud_ptr);

    clouds.emplace_back(trans_cloud_ptr);
    *whole_cloud_ptr += *trans_cloud_ptr;
    std::cout << files[idx] << " loaded. " << std::endl;

//    if (_visualize_individually)
//    {
//      Eigen::Matrix<double,4,1> temp_centroid;
//      pcl::compute3DCentroid(*trans_cloud_ptr, temp_centroid);
//      temp_viewer->setCameraPosition(temp_centroid[0]-140.0, temp_centroid[1], temp_centroid[2]+120.0, temp_centroid[0], temp_centroid[1], temp_centroid[2], 1, 0, 2);
//      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  color_handler(trans_cloud_ptr, "intensity");
//      temp_viewer->updatePointCloud<pcl::PointXYZI> (trans_cloud_ptr, color_handler, "temp cloud");
//
//      temp_viewer->spinOnce(200);
//    }
  }
  std::cout << " \n  total points number: " << whole_cloud_ptr->size() << std::endl;
//  SaveMarkings("markings.txt", clouds);
//  pcl::io::savePCDFileASCII ("whole_cloud.pcd", *whole_cloud_ptr);

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_ptr, "intensity");
  viewer->addPointCloud<pcl::PointXYZI> (whole_cloud_ptr, intensity_distribution, "whole cloud");

  if (!output_pcd_file_name.empty())
  {
    pcl::io::savePCDFileASCII (output_pcd_file_name.c_str(), *whole_cloud_ptr);
    std::cout << "pcd saved: " << output_pcd_file_name << std::endl;
  }

  Eigen::Matrix<double,4,1> centroid;
  pcl::compute3DCentroid(*whole_cloud_ptr, centroid);
//  Eigen::AngleAxisf init_rotation (1.0, Eigen::Vector3f::UnitY ());
//  Eigen::Translation3f init_translation (centroid[0]-15.0, centroid[1], centroid[2]+15.0);
//  extrinsic = (init_translation * init_rotation).matrix ();
  //viewer->setCameraParameters(intrinsic, extrinsic);
  viewer->setCameraPosition(centroid[0]-140.0, centroid[1], centroid[2]+120.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);

//  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_ptr, "intensity");
//  viewer->addPointCloud<pcl::PointXYZI> (whole_cloud_ptr, intensity_distribution, "whole cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");


  viewer->spin();
}

void
  viewCurveMatching(std::vector<std::string>& files)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PointCloud"));
  boost::shared_ptr<pcl::visualization::PCLVisualizer> curve_viewer (new pcl::visualization::PCLVisualizer ("Curves"));

  std::vector<pcl::PointCloud<PointTypeFull/*pcl::PointXYZI*/>::Ptr> clouds;
  pcl::PointCloud<PointTypeFull/*pcl::PointXYZI*/>::Ptr whole_cloud_ptr(new pcl::PointCloud<PointTypeFull/*pcl::PointXYZI*/>);
  pcl::PointCloud<PointTypeFull/*pcl::PointXYZI*/>::Ptr whole_curves_ptr(new pcl::PointCloud<PointTypeFull/*pcl::PointXYZI*/>);
  std::cout << "To show clouds form id:" << _file_start_id << " to id:" << _file_end_id << " with step " << _file_id_step << std::endl;
  for ( uint idx = (_file_id_step > 0) ? (_file_start_id) : (_file_end_id);
        ( _file_id_step > 0 && idx <= _file_end_id ) || ( _file_id_step < 0 && idx >= _file_start_id );
        idx = idx + _file_id_step )
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    std::string fname = files[idx];
    // std::cout << "debug 1 " << fname << std::endl;

    std::string extension ("");
    if (fname.size () <= extension.size ())
      continue;
    //std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      pcl::io::loadPCDFile (fname, *temp_cloud_ptr);
    }

//    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
//    remove_ground(temp_cloud_ptr, tmp_no_ground_ptr, tmp_ground_ptr);

    // pcl::io::loadPCDFile (files[cloud_id], *temp_cloud_ptr);
    Eigen::Matrix4d initial_pose (Eigen::Matrix4d::Identity());
    if (HAVE_GRAPH_INPUT)
    {
      uint pre_query_vertex_id = idx - _file_id_step;
      uint cur_query_vertex_id = idx;
      initial_pose = getGraphPose(optimizerForLoadingInput, cur_query_vertex_id);
    }
    else if (HAVE_GPS_INPUT)
    {
      uint pre_query_pcd_file_idx = idx - _file_id_step;
      uint cur_query_pcd_file_idx = idx;
      initial_pose = getGPSPose(cur_query_pcd_file_idx);
    }
    pcl::transformPointCloud(*temp_cloud_ptr, *trans_cloud_ptr, initial_pose.cast<float>());
    std::cout << "\n" << "Transformation Matrix: \n" << initial_pose << std::endl;

    //Eigen::Affine3f pse_pose = vertex_transformations[i].cast<float>();
    Eigen::Transform<float, 3, Eigen::Affine> pse_pose(initial_pose.cast<float>());
    //std::cout << pse_pose.translation()[0] << " " << pse_pose.translation()[1] << std::endl;
    // Eigen::Quaternion<float> q(pse_pose.rotation());
    //std::cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    viewer->addCoordinateSystem (1.0, pse_pose);
    curve_viewer->addCoordinateSystem (1.0, pse_pose);

//    pcl::PassThrough<pcl::PointXYZI> pass;
//    pass.setInputCloud (trans_cloud_ptr);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (-0.0, 6.0);
//    pass.setFilterLimitsNegative (false);
//    pass.filter (*trans_cloud_ptr);

    pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);

    pcl::copyPointCloud(*trans_cloud_ptr, *cloud_with_normals);
    pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
    ne.setInputCloud (trans_cloud_ptr);
    ne.setSearchMethod (search_tree);
//    ne.setRadiusSearch (0.1);
    ne.setKSearch(_normal_estimation_k); //outdoor:10
    ne.compute (*cloud_with_normals);

    pcl::PointCloud<PointTypeFull>::Ptr curves (new pcl::PointCloud<PointTypeFull>);
    pcl::PassThrough<PointTypeFull> pass;
    pass.setInputCloud (cloud_with_normals);
    pass.setFilterFieldName ("curvature");
    pass.setFilterLimits (_pass_thresh_low, _pass_thresh_high);
    pass.setFilterLimitsNegative (false);
    pass.filter (*curves);
//
//    pcl::PointCloud<PointTypeFull>::Ptr filteredCloud (new pcl::PointCloud<PointTypeFull>);
//    pcl::PointCloud<PointTypeFull>::Ptr removedPoints (new pcl::PointCloud<PointTypeFull>);
//    pcl::StatisticalOutlierRemoval<PointTypeFull> sor;
//    sor.setInputCloud (cloud_with_normals);
//    sor.setMeanK (param1);
//    sor.setStddevMulThresh (param2);
//    sor.setNegative(true);
//    sor.filter (*filteredCloud);
//    sor.setNegative(true);
//    sor.filter (*removedPoints);


    clouds.emplace_back(cloud_with_normals);
    *whole_cloud_ptr += *cloud_with_normals;
    *whole_curves_ptr += *curves;
    std::cout << files[idx] << " loaded. " << std::endl;

    if (!output_pcd_files_prefix.empty())
    {
      char temp[10];
      sprintf(temp, "%d", idx);
      std::string str_idx(temp);

      pcl::PointCloud<PointTypeIO>::Ptr filteredCloud (new pcl::PointCloud<PointTypeIO>);
      pcl::io::savePCDFileASCII (output_pcd_files_prefix + "_" + str_idx + ".pcd", *curves);
      std::cout << "pcd saved: " << output_pcd_file_name << std::endl;
    }

//    if (_visualize_individually)
//    {
//      Eigen::Matrix<double,4,1> temp_centroid;
//      pcl::compute3DCentroid(*trans_cloud_ptr, temp_centroid);
//      temp_viewer->setCameraPosition(temp_centroid[0]-140.0, temp_centroid[1], temp_centroid[2]+120.0, temp_centroid[0], temp_centroid[1], temp_centroid[2], 1, 0, 2);
//      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  color_handler(trans_cloud_ptr, "intensity");
//      temp_viewer->updatePointCloud<pcl::PointXYZI> (trans_cloud_ptr, color_handler, "temp cloud");
//
//      temp_viewer->spinOnce(200);
//    }
  }

  std::cout << " \n  total points number: " << whole_cloud_ptr->size() << std::endl;
//  SaveMarkings("markings.txt", clouds);
//  pcl::io::savePCDFileASCII ("whole_cloud.pcd", *whole_cloud_ptr);

  pcl::visualization::PointCloudColorHandlerGenericField<PointTypeFull/*pcl::PointXYZI*/>  intensity_distribution(whole_cloud_ptr, "intensity");
  viewer->addPointCloud<PointTypeFull/*pcl::PointXYZI*/> (whole_cloud_ptr, intensity_distribution, "whole cloud");
  pcl::visualization::PointCloudColorHandlerGenericField<PointTypeFull/*pcl::PointXYZI*/>  curve_intensity_distribution(whole_curves_ptr, "curvature");
  curve_viewer->addPointCloud<PointTypeFull/*pcl::PointXYZI*/> (whole_curves_ptr, curve_intensity_distribution, "whole curves");

  if (!output_pcd_file_name.empty())
  {
    pcl::io::savePCDFileASCII (output_pcd_file_name.c_str(), *whole_cloud_ptr);
    std::cout << "pcd saved: " << output_pcd_file_name << std::endl;
  }

  Eigen::Matrix<double,4,1> centroid;
  pcl::compute3DCentroid(*whole_cloud_ptr, centroid);
//  Eigen::AngleAxisf init_rotation (1.0, Eigen::Vector3f::UnitY ());
//  Eigen::Translation3f init_translation (centroid[0]-15.0, centroid[1], centroid[2]+15.0);
//  extrinsic = (init_translation * init_rotation).matrix ();
  //viewer->setCameraParameters(intrinsic, extrinsic);
  viewer->setCameraPosition(centroid[0]-140.0, centroid[1], centroid[2]+120.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);
  curve_viewer->setCameraPosition(centroid[0]-140.0, centroid[1], centroid[2]+120.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);

//  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_ptr, "intensity");
//  viewer->addPointCloud<pcl::PointXYZI> (whole_cloud_ptr, intensity_distribution, "whole cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");
  curve_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole curves");

  viewer->spin();
}

void
  viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point_cloud");
  //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0 );

  pcl::ModelCoefficients coeffs;
  // translation
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  // rotation
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  // size
  coeffs.values.push_back (3.5);
  coeffs.values.push_back (1.8);
  coeffs.values.push_back (1.3);
  viewer.removeShape ("cube");
  viewer.addCube (coeffs, "cube");
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,1.0,1.0,"cube");

  //  pcl::PointXYZ o;
  //  o.x = 1.0;
  //  o.y = 0;
  //  o.z = 0;
  //  viewer.addSphere (o, 0.25, "sphere", 0);
  //  std::cout << "i only run once" << std::endl;

}

void
  viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
  static unsigned count = 0;
  std::stringstream ss;
  ss << "Once per viewer loop: " << count++;
  viewer.removeShape ("text", 0);
  viewer.addText (ss.str(), 200, 300, "text", 0);

  //FIXME: possible race condition here:
  user_data++;
}

int 
	main (int argc, char** argv)
{
//	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	//pcl::visualization::PCLPlotter* plotter = new  pcl::visualization::PCLPlotter;

  parseCommandLine(argc, argv);
  std::cout << "commands parsed." << std::endl;

//  // ./viewer --pcd_dir ~/data/pvg2jinke_pcd/ -do_filter --param1 3 --param2 0.34 --output_markings markings_pvg2jinke
//  handlingSGMPointCloud(file_names);

//  view(file_names);

  viewCurveMatching(file_names);

//  ///////////////////////////////////////////////////////////////////////////////////////////////
//  ///////////////////                Creating SGMs                  /////////////////////////////
//  ///////////////////////////////////////////////////////////////////////////////////////////////
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> temp_viewer (new pcl::visualization::PCLVisualizer ("temp Viewer"));
//  pcl::PointCloud<pcl::PointXYZI>::Ptr null_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
//  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  null_handler(null_cloud_ptr, "intensity");
//  temp_viewer->addPointCloud<pcl::PointXYZI> (null_cloud_ptr, null_handler, "temp cloud");
//
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//
//  parseCommandLine(argc, argv);
//  std::cout << "commands parsed." << std::endl;
//
//  // for (int i=0; i<_initial_poses.size();i=i+10)
//  // {
//  //   //std::cout << i <<" . ";
//  //   //Eigen::Affine3f pse_pose = vertex_transformations[i].cast<float>();
//  //   Eigen::Transform<float, 3, Eigen::Affine> pse_pose(vertex_transformations[i].cast<float>());
//  //   //std::cout << pse_pose.translation()[0] << " " << pse_pose.translation()[1] << std::endl;
//  //   Eigen::Quaternion<float> q(pse_pose.rotation());
//  //   //std::cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
//  //   viewer->addCoordinateSystem (1.0, pse_pose);
//
//  //   // char a[10];
//  //   // sprintf(a, "%05d", i);
//  //   // std::string ss("cube");
//  //   // ss.append(a);
//  // }
//
//  pcl::ModelCoefficients coeffs;
//  // translation
//  coeffs.values.push_back (0.0);
//  coeffs.values.push_back (0.0);
//  coeffs.values.push_back (0.0);
//  // rotation
//  coeffs.values.push_back (0.0);
//  coeffs.values.push_back (0.0);
//  coeffs.values.push_back (0.0);
//  coeffs.values.push_back (1.0);
//  // size
//  coeffs.values.push_back (3.5);
//  coeffs.values.push_back (1.8);
//  coeffs.values.push_back (1.3);
//  viewer->removeShape ("cube");
//  viewer->addCube (coeffs, "cube");
//  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
//  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,1.0,1.0,"cube");
//
////	printf("loading");
////	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
////	loadData (file_names, clouds);
////	printf("loaded");
//  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
//  pcl::PointCloud<pcl::PointXYZI>::Ptr whole_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
//	std::cout << "To show clouds form id:" << _file_start_id << " to id:" << _file_end_id << " with step " << _file_id_step << std::endl;
//  for ( uint idx = (_file_id_step > 0) ? (_file_start_id) : (_file_end_id);
//        ( _file_id_step > 0 && idx <= _file_end_id ) || ( _file_id_step < 0 && idx >= _file_start_id );
//        idx = idx + _file_id_step )
//  {
//	  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
//	  pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
//
//    std::string fname = file_names[idx];
//    // std::cout << "debug 1 " << fname << std::endl;
//
//    std::string extension ("");
//    if (fname.size () <= extension.size ())
//      continue;
//    //std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
//    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
//    {
//      // Load the cloud and saves it into the global list of models
//      pcl::io::loadPCDFile (fname, *temp_cloud_ptr);
//    }
//
////    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
////    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
////    remove_ground(temp_cloud_ptr, tmp_no_ground_ptr, tmp_ground_ptr);
//
//    // pcl::io::loadPCDFile (file_names[cloud_id], *temp_cloud_ptr);
//    Eigen::Matrix4d initial_pose (Eigen::Matrix4d::Identity());
//    if (HAVE_GRAPH_INPUT)
//    {
//      uint pre_query_vertex_id = idx - _file_id_step;
//      uint cur_query_vertex_id = idx;
//      initial_pose = getGraphPose(optimizerForLoadingInput, cur_query_vertex_id);
//    }
//    else if (HAVE_GPS_INPUT)
//    {
//      uint pre_query_pcd_file_idx = idx - _file_id_step;
//      uint cur_query_pcd_file_idx = idx;
//      initial_pose = getGPSPose(cur_query_pcd_file_idx);
//    }
//    else if (_remove_z_offset)
//    {
//      Eigen::Vector4f min_pt, max_pt;
//      pcl::getMinMax3D (*temp_cloud_ptr, min_pt, max_pt);
//      std::cout << "min_pt: " << min_pt << std::endl;
//      std::cout << "max_pt: " << max_pt << std::endl;
//      initial_pose(2,3) = -min_pt(2)-2.0;
//    }
//    pcl::transformPointCloud(*temp_cloud_ptr, *trans_cloud_ptr, initial_pose.cast<float>());
//    std::cout << "\n" << "Transformation Matrix: \n" << initial_pose << std::endl;
//
//    //Eigen::Affine3f pse_pose = vertex_transformations[i].cast<float>();
//    Eigen::Transform<float, 3, Eigen::Affine> pse_pose(initial_pose.cast<float>());
//    //std::cout << pse_pose.translation()[0] << " " << pse_pose.translation()[1] << std::endl;
//    // Eigen::Quaternion<float> q(pse_pose.rotation());
//    //std::cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
//    viewer->addCoordinateSystem (1.0, pse_pose);
//
//    pcl::PassThrough<pcl::PointXYZI> pass;
//    pass.setInputCloud (trans_cloud_ptr);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (-0.0, 6.0);
//    pass.setFilterLimitsNegative (false);
//    pass.filter (*trans_cloud_ptr);
//
//    clouds.emplace_back(trans_cloud_ptr);
//	  *whole_cloud_ptr += *trans_cloud_ptr;
//		std::cout << file_names[idx] << " loaded. " << std::endl;
//
////		if (_visualize_individually)
////		{
////		  Eigen::Matrix<double,4,1> temp_centroid;
////		  pcl::compute3DCentroid(*trans_cloud_ptr, temp_centroid);
////		  temp_viewer->setCameraPosition(temp_centroid[0]-140.0, temp_centroid[1], temp_centroid[2]+120.0, temp_centroid[0], temp_centroid[1], temp_centroid[2], 1, 0, 2);
////		  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  color_handler(trans_cloud_ptr, "intensity");
////		  temp_viewer->updatePointCloud<pcl::PointXYZI> (trans_cloud_ptr, color_handler, "temp cloud");
////
////		  temp_viewer->spinOnce(200);
////		}
//	}
//	std::cout << " \n  total points number: " << whole_cloud_ptr->size() << std::endl;
////	SaveMarkings("markings.txt", clouds);
////	pcl::io::savePCDFileASCII ("whole_cloud.pcd", *whole_cloud_ptr);
//
//	pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
////	pcl::copyPointCloud(*wholeCloud,*filteredCloud);
//	pcl::PointCloud<pcl::PointXYZI>::Ptr removedPoints(new pcl::PointCloud<pcl::PointXYZI>);
//	//pcl::PointIndices::Ptr removedPointsIndicesPtr (new pcl::PointIndices);
//	//pcl::PointIndices filteredPointsIndices;
//
//	if (_do_filter)
//	{
//	  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
//	  sor.setInputCloud (whole_cloud_ptr);
//	  sor.setMeanK (param1);
//	  sor.setStddevMulThresh (param2);
//	  sor.setNegative(false);
//	  sor.filter (*filteredCloud);
//	  sor.setNegative(true);
//	  sor.filter (*removedPoints);
//	}
//	else
//	  pcl::copyPointCloud(*whole_cloud_ptr,*filteredCloud);
//
//	std::vector<pcl::PointCloud<PointTypeFull>::Ptr> segmented_component_ptrs;
//	pcl::PointCloud<PointTypeFull>::Ptr pre_useful_cloud_ptr (new pcl::PointCloud<PointTypeFull>);
//	pcl::PointCloud<PointTypeFull>::Ptr segmented_cloud_to_show_ptr (new pcl::PointCloud<PointTypeFull>);
//	segment_cloud(filteredCloud, segmented_component_ptrs, pre_useful_cloud_ptr, segmented_cloud_to_show_ptr);
//
//	pcl::PointCloud<pcl::PointXYZI>::Ptr points_to_visualize(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::copyPointCloud(*segmented_cloud_to_show_ptr,*points_to_visualize);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr plain_whole_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::copyPointCloud(*filteredCloud,*plain_whole_cloud_ptr);
//
//	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(points_to_visualize, "intensity");
//	viewer->addPointCloud<pcl::PointXYZI> (points_to_visualize, intensity_distribution, "whole cloud");
//
//	if (!output_pcd_file_name.empty())
//	{
//	  pcl::io::savePCDFileASCII (output_pcd_file_name.c_str(), *whole_cloud_ptr);
//	  std::cout << "pcd saved: " << output_pcd_file_name << std::endl;
//	}
//
//  if (!output_markings_file_name.empty())
//  {
//    if (!segmented_component_ptrs.empty())
//      SaveMarkings(output_markings_file_name.c_str(), segmented_component_ptrs);
//    else
//      SaveMarkings(output_markings_file_name.c_str(), clouds);
//    std::cout << "markings saved: " << output_markings_file_name << std::endl;
//  }
//
//  Eigen::Matrix<double,4,1> centroid;
//  pcl::compute3DCentroid(*whole_cloud_ptr, centroid);
////  Eigen::AngleAxisf init_rotation (1.0, Eigen::Vector3f::UnitY ());
////  Eigen::Translation3f init_translation (centroid[0]-15.0, centroid[1], centroid[2]+15.0);
////  extrinsic = (init_translation * init_rotation).matrix ();
//  //viewer->setCameraParameters(intrinsic, extrinsic);
//  viewer->setCameraPosition(centroid[0]-140.0, centroid[1], centroid[2]+120.0, centroid[0], centroid[1], centroid[2], 1, 0, 2);
//
////  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  intensity_distribution(whole_cloud_ptr, "intensity");
////  viewer->addPointCloud<pcl::PointXYZI> (whole_cloud_ptr, intensity_distribution, "whole cloud");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "whole cloud");
//
//
////  pcl::PointCloud<pcl::PointXYZI>::Ptr ibeo_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
////  pcl::io::loadPCDFile ("ibeo.pcd", *ibeo_cloud_ptr);
////  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>  ibeo_intensity_distribution(ibeo_cloud_ptr, "intensity");
////  viewer->addPointCloud<pcl::PointXYZI> (ibeo_cloud_ptr, ibeo_intensity_distribution, "ibeo cloud");
////  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ibeo cloud");
//
//
//  viewer->spin();
//  ///////////////////////////////////////////////////////////////////////////////////////////////
//  ///////////////////              end Creating SGMs                /////////////////////////////
//  ///////////////////////////////////////////////////////////////////////////////////////////////

//	printf(" \n  total points number: %d ", wholeCloud->size());
//
///*
//	std::vector<double> idx;
//	std::vector<double> intensity_values;
//	idx.reserve(wholeCloud->size());
//	intensity_values.reserve(wholeCloud->size());
//	double max_v = -10000.0, min_v = 10000.0;
//	for (unsigned int i = 0; i<wholeCloud->size(); i++){
//		//std::cout << intensity_values[i] << std::endl;
//		//max_v = max_v > wholeCloud->points[i].intensity ? max_v : wholeCloud->points[i].intensity;
//		//min_v = min_v < wholeCloud->points[i].intensity ? min_v : wholeCloud->points[i].intensity;
//		//if (wholeCloud->points[i].intensity>2)
//		//{
//		//	wholeCloud->points[i].intensity = 2.111;
//		//	std::cout<< "+";
//		//}
//		//if (wholeCloud->points[i].intensity<0)
//		//{
//		//	wholeCloud->points[i].intensity = 0;
//		//	std::cout<< "-";
//		//}
//		if (wholeCloud->points[i].intensity!=wholeCloud->points[i].intensity) {
//			wholeCloud->points[i].intensity = -0.0;
//			//std::cout << "NaN !!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
//		}
//		idx[i] = i;
//		intensity_values[i] = 1.0/(1.0+exp(-3*wholeCloud->points[i].intensity+5.0));
//		wholeCloud->points[i].intensity = intensity_values[i];
//		//max_v = max_v > intensity_values[i] ? max_v : intensity_values[i];
//		//min_v = min_v < intensity_values[i] ? min_v : intensity_values[i];
//		//std::cout << intensity_values[i] << std::endl;
//		//Sleep(500);
//	}
//
//	//std::cout << "max: " << max_v << " min: " << min_v << std::endl;
//	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> markings;
//
//
//	pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::copyPointCloud(*wholeCloud,*filteredCloud);
//	pcl::PointCloud<pcl::PointXYZI>::Ptr removedPoints(new pcl::PointCloud<pcl::PointXYZI>);
//	//pcl::PointIndices::Ptr removedPointsIndicesPtr (new pcl::PointIndices);
//	//pcl::PointIndices filteredPointsIndices;
//
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
//	sor.setInputCloud (wholeCloud);
//	sor.setMeanK (50);
//	sor.setStddevMulThresh (0.0);
//	sor.setNegative(false);
//	sor.filter (*filteredCloud);
//	sor.setNegative(true);
//	sor.filter (*removedPoints);
//	//pcl::IndicesConstPtr removedPointsIndicesPtr = sor.getRemovedIndices();
//
//
//	//pcl::ExtractIndices<pcl::PointXYZI> idx_extractor;
//	//idx_extractor.setInputCloud(wholeCloud);
//	//idx_extractor.setIndices(removedPointsIndicesPtr);
//	//idx_extractor.setNegative(false);
//	//idx_extractor.filter(*removedPoints);
//
//	std::cout << std::endl;
//	std::cout << "filtered point number: " << filteredCloud->points.size() << std::endl;
//	std::cout << "removed point number: " << removedPoints->size() << std::endl;
//
//
//	pcl::PointCloud<pcl::PointXYZI>::Ptr seg_resampled_cloud (new pcl::PointCloud<pcl::PointXYZI>);
//
//	pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedCloud(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::copyPointCloud(*filteredCloud,*segmentedCloud);
//
//	pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
//	pcl::ConditionalEuclideanClustering<pcl::PointXYZI> cec (true);
//	cec.setInputCloud (segmentedCloud);
//	cec.setConditionFunction (&QSRegionGrowing);
//	cec.setClusterTolerance (0.5);
//	cec.setMinClusterSize (3000); //outdoor:cloud_with_normals->points.size () / 2000
//	cec.setMaxClusterSize (filteredCloud->points.size () / 2);
//	cec.segment (*clusters);
//	cec.getRemovedClusters (small_clusters, large_clusters);
//
//	//// Using the intensity channel for lazy visualization of the output
//	//for (int i = 0; i < small_clusters->size (); ++i){
//	//	for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j){
//	//		segmentedCloud->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
//	//		//current_cloud_i->points.push_back(cloud_out->points[(*small_clusters)[i].indices[j]]);
//	//	}
//	//}
//
//	//for (int i = 0; i < large_clusters->size (); ++i){
//
//	//	for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j){
//	//		segmentedCloud->points[(*large_clusters)[i].indices[j]].intensity = 10.0;
//	//		//current_cloud_i->points.push_back(cloud_out->points[(*large_clusters)[i].indices[j]]);
//	//	}
//	//}
//
//	double rssi = 0;
//	for (int i = 0; i < clusters->size (); ++i)
//	{
//		pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_i (new pcl::PointCloud<pcl::PointXYZI>);
//		//current_cloud_i->reserve((*clusters)[i].indices.size ());
//
//		//int label = rand () % 8;
//		for (int j = 0; j < (*clusters)[i].indices.size (); ++j){
//			current_cloud_i->points.emplace_back(segmentedCloud->points[(*clusters)[i].indices[j]]);
//			current_cloud_i->points.back().intensity = rssi;
//			rssi = rssi+0.01;
//
//			//segmentedCloud->points[(*clusters)[i].indices[j]].intensity = label;
//		}
//
//		pcl::VoxelGrid<pcl::PointXYZI> gridfilter;
//		gridfilter.setInputCloud (current_cloud_i);
//		gridfilter.setLeafSize (3.0f, 3.0f, 3.0f);
//		gridfilter.filter (*current_cloud_i);
//
//		std::cout << "segment " << i << " contains " << current_cloud_i->size() << " points " << std::endl;
//		*seg_resampled_cloud += *current_cloud_i;
//		markings.emplace_back(current_cloud_i);
//	}
//
//	//*seg_resampled_cloud += *removedPoints;
//
//	pcl::PointCloud<pcl::PointXYZI>::Ptr boards (new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::PassThrough<pcl::PointXYZI> pass;
//	pass.setInputCloud (removedPoints);
//	pass.setFilterFieldName ("z");
//	pass.setFilterLimits (2.0, 10.0);
//	//pass.setFilterLimitsNegative (true);
//	pass.filter (*boards);
//
//	pass.setInputCloud (boards);
//	pass.setFilterFieldName ("intensity");
//	pass.setFilterLimits (0.5, 50.0);
//	//pass.setFilterLimitsNegative (true);
//	pass.filter (*boards);
//
//	//pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
//	sor.setInputCloud (boards);
//	sor.setMeanK (20);
//	sor.setStddevMulThresh (0.0);
//	sor.setNegative(false);
//	sor.filter (*boards);
//
//
//	//pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZI>);
//	//pcl::PointCloud<pcl::PointXYZINormal>::Ptr boards_with_normal (new pcl::PointCloud<pcl::PointXYZINormal>);
//	//pcl::copyPointCloud(*boards, *boards_with_normal);
//	//pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;
//	//ne.setInputCloud (boards);
//	//ne.setSearchMethod (search_tree);
//	////ne.setRadiusSearch (300);
//	//ne.setKSearch(10); //outdoor:10
//	//ne.compute (*boards_with_normal);
//
//	//pcl::PassThrough<pcl::PointXYZINormal> pass_normal;
//	//pass_normal.setInputCloud (boards_with_normal);
//	//pass_normal.setFilterFieldName ("normal_z");
//	//pass_normal.setFilterLimits (-0.2, 0.2);
//	////pass.setFilterLimitsNegative (true);
//	//pass_normal.filter (*boards_with_normal);
//
//	//pcl::copyPointCloud(*boards_with_normal, *boards);
//
//
//	//pcl::VoxelGrid<pcl::PointXYZI> gridfilter;
//	//gridfilter.setInputCloud (filteredCloud);
//	//gridfilter.setLeafSize (1.0f, 1.0f, 1.0f);
//	//gridfilter.filter (*filteredCloud);
//
//	//pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
//	//pcl::ConditionalEuclideanClustering<pcl::PointXYZI> cec (true);
//	cec.setInputCloud (boards);
//	cec.setConditionFunction (&QSRegionGrowing);
//	cec.setClusterTolerance (3);
//	cec.setMinClusterSize (10); //outdoor:cloud_with_normals->points.size () / 2000
//	cec.setMaxClusterSize (boards->points.size () / 10);
//	cec.segment (*clusters);
//	cec.getRemovedClusters (small_clusters, large_clusters);
//
//	std::cout << "small_cluster_size: " << small_clusters->size() << " large_cluster_size: " << large_clusters->size()<< std::endl;
//
//	// Using the intensity channel for lazy visualization of the output
//	//for (int i = 0; i < small_clusters->size (); ++i){
//	//	for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j){
//	//		boards->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
//	//		//current_cloud_i->points.push_back(cloud_out->points[(*small_clusters)[i].indices[j]]);
//	//	}
//	//}
//
//	//for (int i = 0; i < large_clusters->size (); ++i){
//
//	//	for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j){
//	//		boards->points[(*large_clusters)[i].indices[j]].intensity = 10.0;
//	//		//current_cloud_i->points.push_back(cloud_out->points[(*large_clusters)[i].indices[j]]);
//	//	}
//	//}
//	double rssi_step = rssi / ((double)clusters->size());
//	for (int i = 0; i < clusters->size (); ++i)
//	{
//		pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_i (new pcl::PointCloud<pcl::PointXYZI>);
//		//current_cloud_i->reserve((*clusters)[i].indices.size ());
//
//		//int label = rand () % 8;
//		for (int j = 0; j < (*clusters)[i].indices.size (); ++j){
//			current_cloud_i->points.emplace_back(boards->points[(*clusters)[i].indices[j]]);
//			current_cloud_i->points.back().intensity = rssi;
//			//boards->points[(*clusters)[i].indices[j]].intensity = label;
//		}
//		rssi = rssi - rssi_step;
//		//pcl::VoxelGrid<pcl::PointXYZI> gridfilter;
//		//gridfilter.setInputCloud (current_cloud_i);
//		//gridfilter.setLeafSize (3.0f, 3.0f, 3.0f);
//		//gridfilter.filter (*current_cloud_i);
//
//		std::cout << "board " << i << " contains " << current_cloud_i->size() << " points " << std::endl;
//		*seg_resampled_cloud += *current_cloud_i;
//		markings.emplace_back(current_cloud_i);
//	}
//
//	//*seg_resampled_cloud += *boards;
//
//	pcl::io::savePCDFileASCII ("whole_cloud.pcd", *seg_resampled_cloud);
//	SaveMarkings("markings.txt", markings);
//*/
//	viewer.showCloud(wholeCloud/*seg_resampled_cloud*/,"point_cloud");
	//blocks until the cloud is actually rendered
	
//	//std::vector<double> testdata;
//	//for (int i = 0; i<1000; i++){
//	//	testdata.push_back(cos(3.1415926535*(double)(i)/180.0));
//	//}
//	//plotter->addPlotData(idx,intensity_values,"intensity",vtkChart::POINTS);
//	//plotter->addHistogramData(intensity_values,100);
//	//plotter->setColorScheme(vtkColorSeries::BLUES);
//	//plotter->plot();
//
//	//use the following functions to get access to the underlying more advanced/powerful
//	//PCLVisualizer
//
//	//This will only get called once
////	viewer.runOnVisualizationThreadOnce (viewerOneOff);
//
//
//	//This will get called once per visualization iteration
//	//viewer.runOnVisualizationThread (viewerPsycho);
//	while (!viewer.wasStopped ())
//	{
//		//plotter->spinOnce (100);
//		//you can also do cool processing here
//		//FIXME: Note that this is running in a separate thread from viewerPsycho
//		//and you should guard against race conditions yourself...
//		user_data++;
//	}
	return 0;
}
