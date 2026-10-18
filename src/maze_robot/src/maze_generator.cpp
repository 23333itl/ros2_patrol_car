//机器人小车 走迷宫 
//1 生成迷宫  命令行工具
//2 机器人小车在迷宫中扫描 进行slam建图  (在未知环境中扫描)
//3 导航 避障(走迷宫)
#include<iostream>
#include<vector>
#include<random> //随机数种子
#include<algorithm>
#include<queue>
#include<string>
#include<fstream>
#include<cmath>
#include<cstdint>
#include <cstdlib>
#include<array>
#include<filesystem>
#include<stdexcept>
#include<utility>


namespace fs=std::filesystem;
using std::string;
using std::vector;

constexpr double kMapMargin=0.5;//地图周围留出0.5米空地

struct Options{
    int rows=10;
    int cols=10;
    double cell_size=0.8;//格子大小
    double wall_thickness=0.08;//墙的厚度
    double wall_height=0.4;
    uint64_t seed=0;
    bool seed_set=false;
    double loop_ratio=0.0;//0-1之间 
    int start_r =0,start_c=0;
    int goal_r=-1,goal_c=-1;//-1代表取对角
    double start_yaw=0.0;//初始朝向
    bool openings=false;
    string name="maze";
    string output_dir=(fs::path(getenv("HOME")) / "maze_ws" / "maze_output").string();
    double map_resolution=0.05;//地图分辨率
    string render_engine="ogre2";//ogre2 支持 gpu_lidar/rgbd_camera；ogre 可作软件渲染兜底
};//

struct Box{
    double cx,cy;//平面矩形中心点的xy坐标
    double sx,sy;//给定中心点坐标和xy方向的长度唯一确定一个矩形
};

struct Marker{
    string name;
    double cx,cy,size;
    std::array<double,4> rgba;
};

struct Maze{
    int rows=0;
    int cols=0;
    vector<vector<bool>> vw;//表示格子垂直左边方向墙壁是否存在
    vector<vector<bool>> hw;//表示格子水平上方方向墙壁是否存在 
    int sr, sc, gr, gc;//起点坐标与终点坐标

};

Maze generateMaze(const Options o,std::mt19937_64& rng){
    Maze m;
    m.cols=o.cols;
    m.rows=o.rows;
    m.hw.assign(o.rows+1,vector<bool>(o.cols,true));
    m.vw.assign(o.rows,vector<bool>(o.cols+1,true));
    m.sr=o.start_r;
    m.sc=o.start_c;
    m.gr=o.goal_r>=0?o.goal_r:o.rows-1;
    m.gc=o.goal_c>=0?o.goal_c:o.cols-1;
    vector<vector<bool>> visited(o.rows,vector<bool>(o.cols,false));
    vector<std::pair<int,int>> stack;
    stack.push_back({m.sr,m.sc});
    visited[m.sr][m.sc]=true;

    const int dr[4]={-1,1,0,0};
    const int dc[4]={0,0,-1,1};

    while(!stack.empty()){
        //1 取出最上面的元素
        //2 有无可走的方向，有就压栈
        //3 没有就pop
        const auto [r,c]=stack.back();
        vector<int> cand;
        for(int k=0;k<4;k++){
            const int nr=r+dr[k],nc=c+dc[k];
            //没有被访问过且可走
            if(nr>=0 && nr<m.rows && nc>=0 && nc<m.cols && !visited[nr][nc]){
                cand.push_back(k);
            }
        }
        if(cand.empty()){
            stack.pop_back();
            continue;
        }

        std::uniform_int_distribution<size_t> pick(0,cand.size()-1);
        const int k=cand[pick(rng)];
        const int nr=r+dr[k],nc=c+dc[k];
        visited[nr][nc]=true;
        if(k==0){
            m.hw[r][c]=false;
        }
        else if(k==1){
            m.hw[r+1][c]=false;
        }
        else if(k==2){
            m.vw[r][c]=false;
        }
        else{
            m.vw[r][c+1]=false;
        }
        stack.push_back({nr,nc});
    }
    if(o.loop_ratio>0.0){
        struct Wall{
            bool horizontal;
            int r,c;
        };
        vector<Wall> standing;
        for(int i=1;i<m.rows;i++){
            for(int j=0;j<m.cols;j++){
                if(m.hw[i][j]){
                    standing.push_back({true,i,j});
                }
            }
        } 
        for(int i=0;i<m.rows;i++){
            for(int j=1;j<m.cols;j++){
                if(m.vw[i][j]){
                    standing.push_back({false,i,j});
                }
            }
        } 
        std::shuffle(standing.begin(),standing.end(),rng);
        const size_t k=static_cast<size_t>(std::llround(o.loop_ratio*standing.size()));
        for(size_t i=0;i<k;i++){
            if(standing[i].horizontal){
                m.hw[standing[i].r][standing[i].c]=false;
            }
            else{
                m.vw[standing[i].r][standing[i].c]=false;
            }
        }
    }
    if(o.openings){
        m.vw[m.sr][m.sc]=false;
        m.vw[m.gr][m.gc+1]=false;
    }
    return m;
}

vector<Box> buildBoxes(const Maze& m,double L,double t){
    vector<Box> boxes;
    for(int i=0;i<=m.rows;i++){
        int c=0;
        while(c<m.cols){
            if(!m.hw[i][c]){
                c++;
                continue;
            }
            int c1=c;
            while(c1+1<m.cols && m.hw[i][c1+1]){
                c1++;
            }
            boxes.push_back({(c+c1+1)/2.0*L,i*L,(c1-c+1)*L+t,t});
            c=c1+1;
        }
    }
    for(int i=0;i<=m.cols;i++){
        int r=0;
        while(r<m.rows){
            if(!m.vw[r][i]){
                r++;
                continue;
            }
            int r1=r;
            while(r1+1<m.rows && m.vw[r1+1][i]){
                r1++;
            }
            boxes.push_back({i*L,(r+r1+1)/2.0*L,t,(r1-r+1)*L+t});
            r=r1+1;
        }
    }
    return boxes;
}

int bfsShortest(const Maze& m){
    auto idx=[&](int r,int c){return r*m.cols+c;};
    vector<int> dist(m.rows*m.cols,-1);
    std::queue<std::pair<int,int>> q;
    dist[idx(m.sr,m.sc)]=0;
    q.push({m.sr,m.sc});
    const int dr[4]={-1,1,0,0};
    const int dc[4]={0,0,-1,1};

    while(!q.empty()){
        const auto [r,c]=q.front();
        q.pop();
        if(r==m.gr&&c==m.gc){
            return dist[idx(r,c)];
        }
        for(int k=0;k<4;k++){
            const int nr=r+dr[k],nc=c+dc[k];
            if(nr<0 || nr>=m.rows || nc<0 || nc>=m.cols){
                continue;
            }
            if(dist[idx(nr,nc)]>=0) continue;
            bool pass;
            if(k==0){
                pass=!m.hw[r][c];
            }
            else if(k==1){
                pass=!m.hw[r+1][c];
            }
            else if(k==2){
                pass=!m.vw[r][c];
            }
            else{
                pass=!m.vw[r][c+1];
            }
            if(!pass) continue;
            dist[idx(nr,nc)]=dist[idx(r,c)]+1;
            q.push({nr,nc});
        }
    }
    return -1;
}

void printUsage() {
  std::cout <<
    R"(用法: maze_generator [选项]

  --rows N            迷宫行数（默认 10）
  --cols N            迷宫列数（默认 10）
  --seed N            随机种子，-1 = 随机（默认 -1）
  --loop-ratio F      按此比例拆除现存内墙以制造回环，取值 0..1
                      （默认 0；约 0.15 即可让沿墙走算法失效）
  --start R,C         起点格子（默认 0,0）
  --goal R,C          终点格子（默认对角格子）
  --openings B        在起点西侧 / 终点东侧的边界墙上开口（默认 false）
  --cell-size M       格子边长 [m]（默认 0.8，通道宽 = 格长 - 墙厚）
  --wall-thickness M  墙厚 [m]（默认 0.08）
  --wall-height M     墙高 [m]（默认 0.4）
  --start-yaw R       起点位姿的初始朝向 [rad]（默认 0.0）
  --name S            输出文件的主名（默认 "maze"）
  --output-dir PATH   输出目录（默认 "."）
  --map-resolution M  占据栅格地图分辨率 [m/格]，0 = 不输出地图（默认 0.05）
  --render-engine S   Gazebo 渲染后端：ogre2（默认）或 ogre（无 GPU 驱动的软渲染兜底）
  --help              显示本帮助
)";
}


bool parseBool(const std::string& v) {
  if (v == "true" || v == "1" || v == "yes" || v == "on") return true;
  if (v == "false" || v == "0" || v == "no" || v == "off") return false;
  throw std::runtime_error("无效的布尔值：'" + v + "'");
}

void parsePair(const std::string& v, int& a, int& b) {
  const auto comma = v.find(',');
  if (comma == std::string::npos) throw std::runtime_error("需要 R,C 格式，实际得到：'" + v + "'");
  a = std::stoi(v.substr(0, comma));
  b = std::stoi(v.substr(comma + 1));
}

void writeMaterial(std::ostream& os, const std::array<double, 4>& c) {
  os << "<material>"
     << "<ambient>" << c[0] << ' ' << c[1] << ' ' << c[2] << ' ' << c[3] << "</ambient>"
     << "<diffuse>" << c[0] << ' ' << c[1] << ' ' << c[2] << ' ' << c[3] << "</diffuse>"
     << "</material>";
}

void writeWorld(const fs::path& path, const Maze& m, const Options& o,
                const std::vector<Box>& boxes, const std::vector<Marker>& markers) {
  std::ofstream w(path);
  if (!w) throw std::runtime_error("无法写入 " + path.string());
  const double h = o.wall_height;
  const double L = o.cell_size;
  const double gx = m.cols * L / 2.0, gy = m.rows * L / 2.0;

  w << "<?xml version=\"1.0\" ?>\n<sdf version=\"1.8\">\n  <world name=\"" << o.name << "\">\n"
    << "    <gravity>0 0 -9.8</gravity>\n"
    <<"     <plugin name=\"gz::sim::systems::Physics\" filename=\"gz-sim-physics-system\">\n"
    <<"     <max_step_size>0.005</max_step_size>\n"
    <<"     <real_time_factor>1.0</real_time_factor>\n"
    <<"     <real_time_update_rate>200</real_time_update_rate>\n"
    << "       </plugin>\n"
    << "    <plugin name=\"gz::sim::systems::UserCommands\" filename=\"gz-sim-user-commands-system\"/>\n"
    << "    <plugin name=\"gz::sim::systems::SceneBroadcaster\" filename=\"gz-sim-scene-broadcaster-system\"/>\n"
    << "    <plugin name=\"gz::sim::systems::Sensors\" filename=\"gz-sim-sensors-system\">\n"
    << "      <render_engine>" << o.render_engine << "</render_engine>\n"
    << "    </plugin>\n"
    << "    <plugin name=\"gz::sim::systems::Imu\" filename=\"gz-sim-imu-system\"/>\n"
    << "    <scene>\n"
    << "      <ambient>0.4 0.4 0.4 1</ambient>\n"
    << "      <background>0.7 0.7 0.7 1</background>\n"
    << "      <shadows>false</shadows>\n"
    << "    </scene>\n"
    << "    <light name=\"sun\" type=\"directional\">\n"
    << "      <cast_shadows>true</cast_shadows>\n"
    << "      <pose>0 0 10 0 0 0</pose>\n"
    << "      <diffuse>0.8 0.8 0.8 1</diffuse>\n"
    << "      <specular>0.2 0.2 0.2 1</specular>\n"
    << "      <attenuation>\n"
    << "        <range>1000</range>\n"
    << "        <constant>0.9</constant>\n"
    << "        <linear>0.01</linear>\n"
    << "        <quadratic>0.001</quadratic>\n"
    << "      </attenuation>\n"
    << "      <direction>-0.5 0.1 -0.9</direction>\n"
    << "    </light>\n";

  // 地面。
  w << "    <model name=\"ground\">\n"
    << "      <static>true</static>\n"
    << "      <link name=\"ground_link\">\n"
    << "        <pose>" << gx << ' ' << gy << " -0.05 0 0 0</pose>\n"
    << "        <collision name=\"collision\"><geometry><box><size>"
    << m.cols * L + 6.0 << ' ' << m.rows * L + 6.0 << " 0.1</size></box></geometry>"
    // 地面显式摩擦：不写 <surface> 时按默认 mu=1.0 处理；
    // 提高到 2.0 增大抓地力，减少轮子打滑
    << "<surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction></surface>"
       "</collision>\n"
    << "        <visual name=\"visual\"><geometry><box><size>"
    << m.cols * L + 6.0 << ' ' << m.rows * L + 6.0 << " 0.1</size></box></geometry>";
  writeMaterial(w, {0.75, 0.75, 0.75, 1.0});
  w << "</visual>\n      </link>\n    </model>\n";

  // 墙体。
  w << "    <model name=\"maze_walls\">\n      <static>true</static>\n";
  for (size_t i = 0; i < boxes.size(); ++i) {
    const Box& b = boxes[i];
    w << "      <link name=\"wall_" << i << "\">\n"
      << "        <pose>" << b.cx << ' ' << b.cy << ' ' << h / 2.0 << " 0 0 0</pose>\n"
      << "        <collision name=\"collision\"><geometry><box><size>"
      << b.sx << ' ' << b.sy << ' ' << h << "</size></box></geometry></collision>\n"
      << "        <visual name=\"visual\"><geometry><box><size>"
      << b.sx << ' ' << b.sy << ' ' << h << "</size></box></geometry>";
    writeMaterial(w, {0.35, 0.42, 0.55, 1.0});
    w << "</visual>\n      </link>\n";
  }
  w << "    </model>\n";

  // 起点 / 终点地面标记：纯 visual、无碰撞，雷达看不到（仅 1cm 高），
  // 但相机能看到 -> 用于"寻找红色终点垫"这类练习。
  w << "    <model name=\"maze_markers\">\n      <static>true</static>\n";
  for (const Marker& mk : markers) {
    w << "      <link name=\"" << mk.name << "\">\n"
      << "        <pose>" << mk.cx << ' ' << mk.cy << " 0.005 0 0 0</pose>\n"
      << "        <visual name=\"visual\"><geometry><box><size>"
      << mk.size << ' ' << mk.size << " 0.01</size></box></geometry>";
    writeMaterial(w, mk.rgba);
    w << "</visual>\n      </link>\n";
  }
  w << "    </model>\n  </world>\n</sdf>\n";
}

void writeInfo(const fs::path& path, const Maze& m, const Options& o,
               uint64_t seed, int shortest, bool map_written) {
  const double L = o.cell_size;
  std::ofstream f(path);
  if (!f) throw std::runtime_error("无法写入 " + path.string());
  f << "# 本文件由 maze_generator 自动生成，请重新生成而不是手工编辑。\n"
    << "name: " << o.name << "\n"
    << "rows: " << m.rows << "\n"
    << "cols: " << m.cols << "\n"
    << "cell_size: " << L << "\n"
    << "wall_thickness: " << o.wall_thickness << "\n"
    << "wall_height: " << o.wall_height << "\n"
    << "seed: " << seed << "\n"
    << "loop_ratio: " << o.loop_ratio << "\n"
    << "start: {x: " << (m.sc + 0.5) * L << ", y: " << (m.sr + 0.5) * L
    << ", yaw: " << o.start_yaw << "}\n"
    << "goal: {x: " << (m.gc + 0.5) * L << ", y: " << (m.gr + 0.5) * L << "}\n"
    << "shortest_path_cells: " << shortest << "\n";
  if (map_written) f << "map_yaml: " << o.name << "_map.yaml\n";
}

void writeMap(const fs::path& pgm_path, const fs::path& yaml_path, const Maze& m,
              const Options& o, const std::vector<Box>& boxes) {
  const double res = o.map_resolution;
  const double L = o.cell_size;
  const int W = static_cast<int>(std::ceil((m.cols * L + 2.0 * kMapMargin) / res));
  const int H = static_cast<int>(std::ceil((m.rows * L + 2.0 * kMapMargin) / res));
  const double ox = -kMapMargin, oy = -kMapMargin;
  std::vector<uint8_t> img(static_cast<size_t>(W) * H, 254);  // 254 = 空闲，0 = 占据
  for (const Box& b : boxes) {
    const int i0 = std::max(0, static_cast<int>(std::floor((b.cx - b.sx / 2.0 - ox) / res)));
    const int i1 = std::min(W - 1, static_cast<int>(std::ceil((b.cx + b.sx / 2.0 - ox) / res)) - 1);
    const int j0 = std::max(0, static_cast<int>(std::floor((b.cy - b.sy / 2.0 - oy) / res)));
    const int j1 = std::min(H - 1, static_cast<int>(std::ceil((b.cy + b.sy / 2.0 - oy) / res)) - 1);
    for (int j = j0; j <= j1; ++j)
      for (int i = i0; i <= i1; ++i) img[static_cast<size_t>(j) * W + i] = 0;
  }
  std::ofstream pgm(pgm_path, std::ios::binary);
  if (!pgm) throw std::runtime_error("无法写入 " + pgm_path.string());
  pgm << "P5\n" << W << ' ' << H << "\n255\n";
  for (int j = H - 1; j >= 0; --j)  // PGM 第 0 行 = 图像顶部 = 最大的 y
    pgm.write(reinterpret_cast<const char*>(&img[static_cast<size_t>(j) * W]), W);

  std::ofstream y(yaml_path);
  if (!y) throw std::runtime_error("无法写入 " + yaml_path.string());
  y << "image: " << o.name << "_map.pgm\n"
    << "resolution: " << res << "\n"
    << "origin: [" << ox << ", " << oy << ", 0.0]\n"
    << "negate: 0\n"
    << "occupied_thresh: 0.65\n"
    << "free_thresh: 0.196\n";
}


int main(int argc,char** argv){
    Options o;
    try {
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      auto value = [&]() -> std::string {
        if (i + 1 >= argc) throw std::runtime_error("参数缺少值：" + a);
        return argv[++i];
      };
      if (a == "--help" || a == "-h") {
        printUsage();
        return 0;
      } 
      else if (a == "--rows") o.rows = std::stoi(value());
      else if (a == "--cols") o.cols = std::stoi(value());
      else if (a == "--cell-size") o.cell_size = std::stod(value());
      else if (a == "--wall-thickness") o.wall_thickness = std::stod(value());
      else if (a == "--wall-height") o.wall_height = std::stod(value());
      else if (a == "--seed") {
        const std::string v = value();
        if (v == "-1") {
          o.seed_set = false;
        } else {
          o.seed = std::stoull(v);
          o.seed_set = true;
        }
      }
      else if (a == "--loop-ratio") o.loop_ratio = std::stod(value());
      else if (a == "--start") parsePair(value(), o.start_r, o.start_c);
      else if (a == "--goal") {
        const std::string v = value();
        if (!v.empty()) parsePair(v, o.goal_r, o.goal_c);
      }
      else if (a == "--start-yaw") o.start_yaw = std::stod(value());
      else if (a == "--openings") o.openings = parseBool(value());
      else if (a == "--name") o.name = value();
      else if (a == "--output-dir")   o.output_dir = value();
      else if (a == "--map-resolution") o.map_resolution = std::stod(value());
      else if (a == "--render-engine") o.render_engine = value();
      else throw std::runtime_error("未知参数：" + a);
    }
    //边界限制
    if (o.cell_size <= 0.0 || o.wall_thickness <= 0.0 || o.wall_height <= 0.0)
      throw std::runtime_error("cell-size / wall-thickness / wall-height 必须为正数");
    if (o.cell_size <= o.wall_thickness)
      throw std::runtime_error("cell-size 必须大于 wall-thickness");
    if (o.render_engine != "ogre" && o.render_engine != "ogre2")
      throw std::runtime_error("render-engine 只支持 ogre 或 ogre2");
    const double corridor = o.cell_size - o.wall_thickness;
    if (corridor < 0.3)
      std::cerr << "警告：通道宽 " << corridor
                << " m，对约 0.2 m 宽的机器人来说太窄了\n";
    if (o.rows < 1 || o.cols < 1)
      throw std::runtime_error("rows 和 cols 必须 >= 1");
    if (o.loop_ratio < 0.0 || o.loop_ratio > 1.0)
      throw std::runtime_error("loop-ratio 必须在 [0, 1] 范围内");
    auto in_range = [&](int r, int c) {
      return r >= 0 && r < o.rows && c >= 0 && c < o.cols;
    };
    if (!in_range(o.start_r, o.start_c)) throw std::runtime_error("起点格子超出范围");
    const int gr = (o.goal_r >= 0) ? o.goal_r : o.rows - 1;
    const int gc = (o.goal_c >= 0) ? o.goal_c : o.cols - 1;
    if (!in_range(gr, gc)) throw std::runtime_error("终点格子超出范围");
    if (gr == o.start_r && gc == o.start_c)
      throw std::runtime_error("起点和终点不能是同一个格子");

    // 未指定 seed 时用真随机数，并打印出来以便复现（考试迷宫可回放）。
    const uint64_t seed = o.seed_set
        ? o.seed
        : (static_cast<uint64_t>(std::random_device{}()) << 32) | std::random_device{}();
    std::mt19937_64 rng(seed);

    Maze m = generateMaze(o, rng);
    const int shortest = bfsShortest(m);
    if (shortest < 0)
      std::cerr << "警告：该迷宫中起点无法到达终点\n";

    fs::create_directories(o.output_dir);
    const fs::path out(o.output_dir);
    const std::vector<Box> boxes = buildBoxes(m, o.cell_size, o.wall_thickness);
    const double L = o.cell_size;
    //起点和终点的标记
    const std::vector<Marker> markers = {
      {"start_marker", (m.sc + 0.5) * L, (m.sr + 0.5) * L, 0.6 * L, {0.1, 0.7, 0.2, 1.0}},
      {"goal_marker", (m.gc + 0.5) * L, (m.gr + 0.5) * L, 0.6 * L, {0.8, 0.1, 0.1, 1.0}},
    };

    writeWorld(out / (o.name + ".world"), m, o, boxes, markers);
    const bool map_written = o.map_resolution > 0.0;
    if (map_written)
      writeMap(out / (o.name + "_map.pgm"), out / (o.name + "_map.yaml"), m, o, boxes);
    writeInfo(out / (o.name + "_info.yaml"), m, o, seed, shortest, map_written);

    std::cout << "迷宫 '" << o.name << "'：" << m.rows << "x" << m.cols
              << " 格，格长 " << o.cell_size << " m，通道宽 "
              << o.cell_size - o.wall_thickness << " m，种子 " << seed << "\n"
              << "  墙体已合并为 " << boxes.size() << " 个 box\n"
              << "  起点格 (" << m.sr << "," << m.sc << ") -> 世界坐标 ("
              << (m.sc + 0.5) * L << ", " << (m.sr + 0.5) * L << ")，终点格 ("
              << m.gr << "," << m.gc << ") -> 世界坐标 (" << (m.gc + 0.5) * L << ", "
              << (m.gr + 0.5) * L << ")\n"
              << "  最短路径：" << shortest << " 格步\n"
              << "  已写出 " << (out / (o.name + ".world")).string() << "\n"
              << "  已写出 " << (out / (o.name + "_info.yaml")).string() << "\n";
    if (map_written)
      std::cout << "  已写出 " << (out / (o.name + "_map.pgm")).string() << "\n"
                << "  已写出 " << (out / (o.name + "_map.yaml")).string() << "\n";
  } catch (const std::exception& e) {
    std::cerr << "错误：" << e.what() << "\n\n";
    printUsage();
    return 1;
  }
  return 0;
}
