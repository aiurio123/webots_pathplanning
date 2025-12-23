/*
 * src/path_planner.c - 全局路径规划器
 *
 * 功能概述：
 *  - 在栅格上使用类似 D* 的算法搜索粗路径（八连通）。
 *  - 对每段使用局部人工势场（APF）进行细化，生成连续航点供控制器使用。
 *  - 输出：waypoints[][4]（x,y,z,yaw）与 waypoint_count。
 *  - 设计说明：限制每次调用的迭代数以避免阻塞实时循环。
 *
 * 注意：
 *  - 网格参数由 GRID_RES / GRID_W / GRID_H 定义。
 *  - occ[][] 由 obstacles_list 填充（将障碍近似为圆形）。
 */
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#define MAX_WAYPOINTS 10000
#include "obstacles.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GRID_RES    0.1 // 栅格分辨率（米）
#define GRID_W      20   // 网格宽（单元格数）
#define GRID_H      20   // 网格高（单元格数）

#define INF_COST    1e9

// neighbor offsets (8-neighborhood)
static const int NEI[8][2] = {
    {1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {1,-1}, {-1,1}, {-1,-1}
};

// APF 参数（局部）
#define K_ATT      9// 吸引力系数
#define K_REP      9 // 排斥力系数
#define REP_RANGE  0.3 // 排斥感应范围（m）
#define STEP_SIZE  0.015   // 每步最小移动距离（m）


#define MAX_NODES (GRID_W*GRID_H)


// output
double waypoints[MAX_WAYPOINTS][4];
int waypoint_count = 0;

static unsigned char occ[GRID_W][GRID_H]; // 0 free, 1 occupied

typedef struct Key { double k1,k2; } Key;
typedef struct DNode {
    int x,y;
    double g,rhs;
    Key key;
    int in_open;
} DNode;

static DNode nodes[GRID_W][GRID_H];

static int open_heap[MAX_NODES+5];
static int open_heap_size = 0;

/* 将世界坐标 (x,y) 转换为栅格索引 (gx,gy)。
 * 若点在网格外返回 0，否则返回 1 并通过指针填充 gx/gy。
 */
static inline int world_to_grid_safe(double x,double y,int *gx,int *gy){
    int cx = GRID_W/2;
    int cy = GRID_H/2;
    double x0=0.8, y0=0.8;
    long gx_l = cx + (long)round((x-x0)/GRID_RES);
    long gy_l = cy + (long)round((y-y0)/GRID_RES);
    if(gx_l<0||gx_l>=GRID_W||gy_l<0||gy_l>=GRID_H) return 0;
    *gx=(int)gx_l; *gy=(int)gy_l; return 1;
}

/* 将栅格索引 (gx,gy) 转换为世界坐标 (x,y)。 */
static inline void grid_to_world(int gx,int gy,double *x,double *y){
    int cx = GRID_W/2;
    int cy = GRID_H/2;
    double x0=0.8, y0=0.8;
    *x = x0 + (gx-cx)*GRID_RES;
    *y = y0 + (gy-cy)*GRID_RES;
}

/* 将 (x,y) 打包为单个 int（高 16 位 x，低 16 位 y），用于堆存储。 */
static inline int pack_xy(int x,int y){ return (x<<16)|(y&0xFFFF); }
/* 从打包的 int 中恢复 (x,y)。 */
static inline void unpack_xy(int v,int *x,int *y){ *x=v>>16; *y=v&0xFFFF; }
/* 比较两个 Key，若 a < b 返回真（用于堆排序）。 */
static int key_less(Key a,Key b){
    if(a.k1<b.k1-1e-9) return 1;
    if(a.k1>b.k1+1e-9) return 0;
    return a.k2<b.k2-1e-9;
}

/* 堆基础操作（用于 open 列表的二叉堆实现）。 */
static void heap_swap(int i,int j){ int t=open_heap[i]; open_heap[i]=open_heap[j]; open_heap[j]=t; }

/* 上浮操作：保持堆的有序性 */
static void heap_up(int idx){
    while(idx>1){
        int p=idx/2;
        int vi=open_heap[idx], vp=open_heap[p];
        int xi,yi,xp,yp; unpack_xy(vi,&xi,&yi); unpack_xy(vp,&xp,&yp);
        if(key_less(nodes[xi][yi].key,nodes[xp][yp].key)){ heap_swap(idx,p); idx=p; } else break;
    }
}

/* 下沉操作：保持堆的有序性 */
static void heap_down(int idx){
    while(1){
        int l=idx*2, r=l+1, smallest=idx;
        if(l<=open_heap_size){
            int vl=open_heap[l]; int xs,ys,xl,yl; unpack_xy(open_heap[smallest],&xs,&ys); unpack_xy(vl,&xl,&yl);
            if(key_less(nodes[xl][yl].key,nodes[xs][ys].key)) smallest=l;
        }
        if(r<=open_heap_size){
            int vr=open_heap[r]; int xs,ys,xr,yr; unpack_xy(open_heap[smallest],&xs,&ys); unpack_xy(vr,&xr,&yr);
            if(key_less(nodes[xr][yr].key,nodes[xs][ys].key)) smallest=r;
        }
        if(smallest!=idx){ heap_swap(idx,smallest); idx=smallest; } else break;
    }
}

/* open 列表（优先队列）接口：初始化 */
static void open_init(){ open_heap_size=0; }
/* 将节点加入 open 列表（若已存在则忽略）。 */
static void open_push(int packed){
    int x,y; unpack_xy(packed,&x,&y);
    if(nodes[x][y].in_open) return;
    open_heap_size++; open_heap[open_heap_size]=packed; nodes[x][y].in_open=1; heap_up(open_heap_size);
} 
/* 从 open 列表弹出优先级最高的节点（若为空返回 -1）。 */
static int open_pop(){ 
    if(open_heap_size==0) return -1;
    int ret=open_heap[1]; int x=ret>>16, y=ret&0xFFFF;
    nodes[x][y].in_open=0;
    open_heap[1]=open_heap[open_heap_size]; open_heap_size--; if(open_heap_size>=1) heap_down(1);
    return ret;
} 

static double km=0.0;
static int sx,sy,gx,gy;

/* 调试输出与辅助函数已移除以减小体积；需要调试时可添加针对性打印或条件宏。 */

static double heuristic_d(int x1,int y1,int x2,int y2){
    double dx=(x1-x2)*GRID_RES; double dy=(y1-y2)*GRID_RES;
    return sqrt(dx*dx+dy*dy);
}

static Key calculateKey(int x,int y){
    double val=fmin(nodes[x][y].g,nodes[x][y].rhs);
    Key k; k.k1=val+heuristic_d(sx,sy,x,y)+km; k.k2=val; return k;
}

static double cost_between(int x1,int y1,int x2,int y2){
    if(x2<0||x2>=GRID_W||y2<0||y2>=GRID_H) return INF_COST;
    if(occ[x2][y2]) return INF_COST;
    double dx=(x1-x2)*GRID_RES; double dy=(y1-y2)*GRID_RES;
    return sqrt(dx*dx+dy*dy);
}

/* 更新节点的 rhs（通过邻居的 g 值计算）并在不一致时将其加入 open。 */
static void updateNode(int x, int y){
    if(x<0 || x>=GRID_W || y<0 || y>=GRID_H) return;
    if(x==gx && y==gy) return;

    double best = INF_COST;

    for(int i=0;i<8;i++){
        int nx = x + NEI[i][0];
        int ny = y + NEI[i][1];
        if(nx<0 || nx>=GRID_W || ny<0 || ny>=GRID_H) continue;
        double c = cost_between(x,y,nx,ny);
        if(c >= INF_COST) continue;
        double val = c + nodes[nx][ny].g;
        if(val < best) best = val;
    }

    nodes[x][y].rhs = best;
    nodes[x][y].key = calculateKey(x,y);

    if(nodes[x][y].g != nodes[x][y].rhs) {
        open_push(pack_xy(x,y));
    }
}



/* 初始化 D*：设置起点/目标、清空 open 列表并初始化节点值 */
static void init_dstar(int start_x,int start_y,int goal_x,int goal_y){
    sx=start_x; sy=start_y; gx=goal_x; gy=goal_y; km=0.0; open_init();
    for(int i=0;i<GRID_W;i++) for(int j=0;j<GRID_H;j++){
        nodes[i][j].x=i; nodes[i][j].y=j; nodes[i][j].g=INF_COST; nodes[i][j].rhs=INF_COST; nodes[i][j].in_open=0;
    }
    nodes[gx][gy].rhs=0.0;
    nodes[gx][gy].key=calculateKey(gx,gy);
    open_push(pack_xy(gx,gy));
}

/* 主搜索循环：处理 open 列表中的节点，最多处理 MAX_ITER_PER_CALL 个节点以避免阻塞仿真。实现 D* 的松弛与堆更新逻辑。 */
static void computeShortestPath() {

// 最大每次扫描节点数（避免阻塞仿真）
const int MAX_ITER_PER_CALL = 300;

int iter = 0;

while (open_heap_size > 0 && iter < MAX_ITER_PER_CALL) {

    // heap top
    int top = open_heap[1];
    int ux = top >> 16;
    int uy = top & 0xFFFF;

    // 取旧 key
    Key k_old = nodes[ux][uy].key;

    // 计算新 key
    Key k_new = calculateKey(ux, uy);

    // 如果新 key 更小，更新堆并退出本次
    if (key_less(k_new, k_old)) {
        nodes[ux][uy].key = k_new;
        heap_down(1);
        iter++;
        continue;
    }

    // 更新 g
    if (nodes[ux][uy].g > nodes[ux][uy].rhs) {
        nodes[ux][uy].g = nodes[ux][uy].rhs;
    } else {
        nodes[ux][uy].g = INF_COST;
    }

    // 弹出堆顶
    open_pop();

    // 八方向邻居（使用 NEI 表）
    for (int i = 0; i < 8; i++) {
        int nx = ux + NEI[i][0];
        int ny = uy + NEI[i][1];
        updateNode(nx, ny);
    }

    iter++;

    // 提前退出：如果起点已经满足条件，就可以结束
    if (open_heap_size > 0) {
        Key startKey = calculateKey(sx, sy);
        int heap_top = open_heap[1];
        int tx = heap_top >> 16;
        int ty = heap_top & 0xFFFF;

        if (!key_less(nodes[tx][ty].key, startKey) &&
            nodes[sx][sy].g <= nodes[sx][sy].rhs) {
            break;
        }
    }
}

}


/* 从起点沿 g 值梯度提取路径，返回路径长度，若失败返回 0。 */
static int extract_path(int path_gx[],int path_gy[],int max_path){
    int px=sx, py=sy, len=0;
    if(nodes[px][py].g>=INF_COST) return 0;
    while(!(px==gx && py==gy) && len<max_path){
        int neighbor_offsets_index = 0; /* use NEI table */
        path_gx[len]=px; path_gy[len]=py; len++;
        double best=INF_COST; int bx=-1, by=-1;
        for(int i=0;i<8;i++){
            int nx = px + NEI[i][0];
            int ny = py + NEI[i][1];
            if(nx<0||nx>=GRID_W||ny<0||ny>=GRID_H) continue;
            double c = cost_between(px,py,nx,ny);
            if(c>=INF_COST) continue;
            double val = c + nodes[nx][ny].g;
            if(val<best){ best=val; bx=nx; by=ny; }
        }
        if(bx<0) return 0;
        px=bx; py=by;
    }
    if(len<max_path){ path_gx[len]=gx; path_gy[len]=gy; len++; }
    return len;
}

/* 更新单个栅格（占用/释放），更新 km 值并调用 updateNode 触发重规划。 */
static void dstar_update_cell(int x,int y,unsigned char new_occ){
    if(x<0||x>=GRID_W||y<0||y>=GRID_H) return;
    unsigned char old=occ[x][y];
    if(old==new_occ) return;
    occ[x][y]=new_occ;
    km+=heuristic_d(sx,sy,x,y);
    updateNode(x,y);
}

#define APF_EXIT_RANGE 0.05      // 距目标5cm内停止 APF

/* 使用人工势场（APF）在 start->goal 之间进行局部细化，生成连续航点并写入全局 waypoints 数组。
 * 保护措施：最大迭代步数、waypoint 容量检查、yaw 的数值有效性检查，防止 NaN/溢出。
 */
static void apf_refine_segment_with_imu(double start[3], double goal[3], Obstacle *obs, int obs_count, WbDeviceTag imu) {
    double pos[4];
    pos[0]=start[0]; pos[1]=start[1]; pos[2]=start[2];
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    pos[3] = rpy ? rpy[2] : 0.0;

    int local_steps=0;
    while (local_steps < 10000 && waypoint_count < MAX_WAYPOINTS) {
        
        double dist_to_goal =
    sqrt((goal[0]-pos[0])*(goal[0]-pos[0]) +
         (goal[1]-pos[1])*(goal[1]-pos[1]) +
         (goal[2]-pos[2])*(goal[2]-pos[2]));

if (dist_to_goal < APF_EXIT_RANGE) {
    // 距离足够近，不再使用 APF - 插入终点前做边界检查
    if (waypoint_count < MAX_WAYPOINTS) {
        waypoints[waypoint_count][0] = goal[0];
        waypoints[waypoint_count][1] = goal[1];
        waypoints[waypoint_count][2] = goal[2];
        double yaw_to_store = pos[3];
        if (!isfinite(yaw_to_store)) yaw_to_store = 0.0;
        waypoints[waypoint_count][3] = yaw_to_store;
        waypoint_count++;
    }
    break;
}

        double dxg = goal[0]-pos[0]; double dyg = goal[1]-pos[1]; double dzg = goal[2]-pos[2];
        double f_att[3] = { K_ATT * dxg, K_ATT * dyg, K_ATT * dzg };
        double f_rep[3] = {0,0,0};
        for (int i=0;i<obs_count;i++){
            double dx = pos[0] - obs[i].x; double dy = pos[1] - obs[i].y; double dz = pos[2] - obs[i].z;
            double dist2d = sqrt(dx*dx + dy*dy);
            if (dist2d < REP_RANGE && dist2d > 1e-6) {
                double rep = K_REP * (1.0/dist2d - 1.0/REP_RANGE);
                rep = rep * rep; 
                f_rep[0] += rep * (dx / dist2d);
                f_rep[1] += rep * (dy / dist2d);
            }
        }
        double f_total[3] = { f_att[0] + f_rep[0], f_att[1] + f_rep[1], f_att[2] + f_rep[2] };
        double norm = sqrt(f_total[0]*f_total[0] + f_total[1]*f_total[1] + f_total[2]*f_total[2]);
        if (norm < 1e-7) break;
        pos[0] += (f_total[0]/norm) * STEP_SIZE;
        pos[1] += (f_total[1]/norm) * STEP_SIZE;
        pos[2] += (f_total[2]/norm) * STEP_SIZE;
        if (pos[2] < 0.3) pos[2] = 0.3;

const double *rpy_now = wb_inertial_unit_get_roll_pitch_yaw(imu);
double current_yaw = rpy_now ? rpy_now[2] : 0.0;

double desired_yaw = atan2(goal[1]-pos[1], goal[0]-pos[0]);
double yaw_err = desired_yaw - current_yaw;

        while (yaw_err > M_PI) yaw_err -= 2*M_PI;
        while (yaw_err < -M_PI) yaw_err += 2*M_PI;


        if (waypoint_count < MAX_WAYPOINTS) {
            waypoints[waypoint_count][0] = pos[0];
            waypoints[waypoint_count][1] = pos[1];
            waypoints[waypoint_count][2] = pos[2];
            double yaw_to_store = current_yaw;
            if (!isfinite(yaw_to_store)) yaw_to_store = 0.0;
            waypoints[waypoint_count][3] = yaw_to_store;
            waypoint_count++;
        } else {
            break; // waypoint buffer 已满
        }

        double dgoal = sqrt((goal[0]-pos[0])*(goal[0]-pos[0]) + (goal[1]-pos[1])*(goal[1]-pos[1]) + (goal[2]-pos[2])*(goal[2]-pos[2]));
        if (dgoal < 0.1) break;
        local_steps++;
    }
}


/* 顶层入口：从 GPS/IMU 读取起点，构建占用栅格，然后使用 D* + APF 生成最终航点序列（waypoints）。
 * 逻辑要点：对异常情况（如起点/目标被包围或不可达）回退为仅 APF 策略。
 */
void init_global_path(WbDeviceTag gps, WbDeviceTag imu, double goal[4], int segments) {
    const double *gps_pos = wb_gps_get_values(gps);
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double start_world[3] = { gps_pos[0], gps_pos[1], gps_pos[2] };
    double goal_world[3] = { goal[0], goal[1], goal[2] };

    // 加载障碍物 -----------------------------------
    load_default_obstacles();

    for (int i=0;i<GRID_W;i++) for (int j=0;j<GRID_H;j++) occ[i][j]=0;
    for (int i=0;i<obs_count;i++) {
                int gx,gy;
        if (!world_to_grid_safe(obstacles_list[i].x, obstacles_list[i].y, &gx, &gy)) {
            /* 障碍物超出边界，跳过 */
        } else {
            int r = (int)ceil(0.2 / GRID_RES);
            for (int ix = gx-r; ix <= gx+r; ix++) for (int jy = gy-r; jy <= gy+r; jy++) {
                if (ix<0||ix>=GRID_W||jy<0||jy>=GRID_H) continue;
                int dx = ix-gx; int dy = jy-gy; if (dx*dx+dy*dy <= r*r) occ[ix][jy]=1;
            }
        }
    }

    int start_gx, start_gy, goal_gx, goal_gy;
    if (!world_to_grid_safe(start_world[0], start_world[1], &start_gx, &start_gy)) {
        int cx = GRID_W/2, cy = GRID_H/2;
        start_gx = cx + (int)round((start_world[0]-0.8)/GRID_RES);
        start_gy = cy + (int)round((start_world[1]-0.8)/GRID_RES);
    }
    if (!world_to_grid_safe(goal_world[0], goal_world[1], &goal_gx, &goal_gy)) {
        int cx = GRID_W/2, cy = GRID_H/2;
        goal_gx = cx + (int)round((goal_world[0]-0.8)/GRID_RES);
        goal_gy = cy + (int)round((goal_world[1]-0.8)/GRID_RES);
    }

    if (start_gx < 0) start_gx = 0; if (start_gx >= GRID_W) start_gx = GRID_W-1;
    if (start_gy < 0) start_gy = 0; if (start_gy >= GRID_H) start_gy = GRID_H-1;
    if (goal_gx < 0) goal_gx = 0; if (goal_gx >= GRID_W) goal_gx = GRID_W-1;
    if (goal_gy < 0) goal_gy = 0; if (goal_gy >= GRID_H) goal_gy = GRID_H-1;

    double dbg_sx, dbg_sy, dbg_gx, dbg_gy;
    grid_to_world(start_gx, start_gy, &dbg_sx, &dbg_sy);
    grid_to_world(goal_gx, goal_gy, &dbg_gx, &dbg_gy);

    if (occ[start_gx][start_gy]) {
        int found = 0;
        for (int r=1; r<=3 && !found; r++) {
            for (int dx=-r; dx<=r && !found; dx++) for (int dy=-r; dy<=r && !found; dy++) {
                int nx = start_gx + dx; int ny = start_gy + dy;
                if (nx<0||nx>=GRID_W||ny<0||ny>=GRID_H) continue;
                if (!occ[nx][ny]) { start_gx = nx; start_gy = ny; found = 1; }
            }
        }
        if (!found) { printf("[D* ERROR] no free neighbor near start!\n"); }
    }
    if (occ[goal_gx][goal_gy]) {
        int found = 0;
        for (int r=1; r<=3 && !found; r++) {
            for (int dx=-r; dx<=r && !found; dx++) for (int dy=-r; dy<=r && !found; dy++) {
                int nx = goal_gx + dx; int ny = goal_gy + dy;
                if (nx<0||nx>=GRID_W||ny<0||ny>=GRID_H) continue;
                if (!occ[nx][ny]) { goal_gx = nx; goal_gy = ny; found = 1; }
            }
        }
        if (!found) { printf("[D* ERROR] no free neighbor near goal!\n"); }
    }

    int free_cnt = 0, occ_cnt = 0;
    for (int ix=0; ix<GRID_W; ix++) for (int jy=0; jy<GRID_H; jy++) {
        if (occ[ix][jy]) occ_cnt++; else free_cnt++;
    }
    /* 统计网格中空闲/占用单元数（用于诊断，可根据需要输出） */


    int cnt_free_around_start = 0;
    for (int dx=-1; dx<=1; dx++) for (int dy=-1; dy<=1; dy++) {
        int nx = start_gx + dx; int ny = start_gy + dy;
        if (nx<0||nx>=GRID_W||ny<0||ny>=GRID_H) continue;
        if (!occ[nx][ny]) cnt_free_around_start++;
    }
    int cnt_free_around_goal = 0;
    for (int dx=-1; dx<=1; dx++) for (int dy=-1; dy<=1; dy++) {
        int nx = goal_gx + dx; int ny = goal_gy + dy;
        if (nx<0||nx>=GRID_W||ny<0||ny>=GRID_H) continue;
        if (!occ[nx][ny]) cnt_free_around_goal++;
    }
    if (cnt_free_around_start == 0 || cnt_free_around_goal == 0) {
        printf("[D* WARN] start or goal is fully surrounded by obstacles (free neighs: start=%d goal=%d). Falling back to APF.\n",
                cnt_free_around_start, cnt_free_around_goal);
        waypoint_count = 0;
        double ss[3] = { start_world[0], start_world[1], start_world[2] };
        apf_refine_segment_with_imu(ss, goal_world, obstacles_list, obs_count, imu);
        /* APF 回退生成航点（不再打印统计信息） */
        return;
    }

    init_dstar(start_gx, start_gy, goal_gx, goal_gy);
    computeShortestPath();


    double start_cost = nodes[sx][sy].g;
    /* 已计算起点代价（用于判断是否可达） */

    
    int path_gx[GRID_W*GRID_H]; int path_gy[GRID_W*GRID_H];
    int path_len = extract_path(path_gx, path_gy, GRID_W*GRID_H);
    if (path_len==0) {
        /* 初始路径无法生成，回退为仅使用 APF 生成航点 */
        waypoint_count = 0;
        double ss[3] = { start_world[0], start_world[1], start_world[2] };
        apf_refine_segment_with_imu(ss, goal_world, obstacles_list, obs_count, imu);
        return; 
    } else {
        // D* 成功找到路径
    }

    double coarse_nodes[ MAX_WAYPOINTS ][3]; int coarse_count=0;
    for (int i=0;i<path_len && coarse_count < MAX_WAYPOINTS; i++) {
        double wx, wy; grid_to_world(path_gx[i], path_gy[i], &wx, &wy);
        coarse_nodes[coarse_count][0] = wx;
        coarse_nodes[coarse_count][1] = wy;
        double t = (double)i / (double)(path_len-1);
        coarse_nodes[coarse_count][2] = start_world[2] + t*(goal_world[2]-start_world[2]);
        coarse_count++;
    }
    

    waypoint_count = 0;
    double seg_start[3] = { start_world[0], start_world[1], start_world[2] };
    for (int i=0;i<coarse_count;i++) {
        double seg_goal[3] = { coarse_nodes[i][0], coarse_nodes[i][1], coarse_nodes[i][2] };
        apf_refine_segment_with_imu(seg_start, seg_goal, obstacles_list, obs_count, imu);

        if (waypoint_count>0) {
            double lastx = waypoints[waypoint_count-1][0];
            double lasty = waypoints[waypoint_count-1][1];
            for (int oi=0; oi<obs_count; oi++) {
                double dx = lastx - obstacles_list[oi].x; double dy = lasty - obstacles_list[oi].y; double d = sqrt(dx*dx + dy*dy);
                if (d < 0.1) {
                    int ox, oy;
                    if (world_to_grid_safe(obstacles_list[oi].x, obstacles_list[oi].y, &ox, &oy)) {
                        dstar_update_cell(ox,oy,1);
                        computeShortestPath();
                    } else {
                        // 障碍物于网格外; 跳过动态更新
                    }
                    int new_path_gx[GRID_W*GRID_H]; int new_path_gy[GRID_W*GRID_H];
                    int cur_sx, cur_sy;
                    if (world_to_grid_safe(lastx, lasty, &cur_sx, &cur_sy)) { sx = cur_sx; sy = cur_sy; }
                    int new_len = extract_path(new_path_gx, new_path_gy, GRID_W*GRID_H);
                    if (new_len > 0) {
                        int idx=0;
                        for (; idx<new_len && idx<MAX_WAYPOINTS; idx++) {
                            double wx,wy; grid_to_world(new_path_gx[idx], new_path_gy[idx], &wx, &wy);
                            coarse_nodes[idx][0]=wx; coarse_nodes[idx][1]=wy;
                            coarse_nodes[idx][2]= start_world[2] + ((double)idx/(new_len-1))*(goal_world[2]-start_world[2]);
                        }
                        coarse_count = new_len;
                        seg_start[0]=lastx; seg_start[1]=lasty; seg_start[2]=start_world[2];
                        i = -1; 
                        break;
                    }
                }
            }
        }
        seg_start[0]=seg_goal[0]; seg_start[1]=seg_goal[1]; seg_start[2]=seg_goal[2];
        if (waypoint_count >= MAX_WAYPOINTS-1) break;
    }

    /* 路径细化完成，生成的航点数量保存在 waypoint_count 中 */
}