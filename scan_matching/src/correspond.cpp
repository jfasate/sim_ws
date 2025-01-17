#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include <limits>


using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point> &old_points, vector<Point> &trans_points, vector<Point> &points,
                            vector<vector<int>> &jump_table, vector<Correspondence> &c, float prob)
{

  c.clear();
  int last_best = -1;
  const int n = trans_points.size();
  const int m = old_points.size();
  int min_index = 0;
  int second_min_index = 0;

  //Do for each point
  for (int ind_trans = 0; ind_trans < n; ++ind_trans)
  {
    float min_dist = 100000.00;
    for (int ind_old = 0; ind_old < m; ++ind_old)
    {
      float dist = old_points[ind_trans].distToPoint2(&trans_points[ind_old]);
      if (dist < min_dist)
      {
        min_dist = dist;
        min_index = ind_old;
        if (ind_old == 0)
        {
          second_min_index = ind_old + 1;
        }
        else
        {
          second_min_index = ind_old - 1;
        }
      }
    }
    c.push_back(Correspondence(&trans_points[ind_trans], &points[ind_trans], &old_points[min_index], &old_points[second_min_index]));
  }
}

void getCorrespondence(vector<Point> &old_points, vector<Point> &trans_points, vector<Point> &points,
                       vector<vector<int>> &jump_table, vector<Correspondence> &c, float prob)
{

  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // use helper functions and structs in transform.h and correspond.h
  // input : old_points : vector of struct points containing the old points (points of the previous frame)
  // input : trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
  // input : points : vector of struct points containing the new points
  // input : jump_table : jump table computed using the helper functions from the transformed and old points
  // input : c: vector of struct correspondences . This is a refernece which needs to be updated in place and return the new correspondences to calculate the transforms.
  // output : c; update the correspondence vector in place which is provided as a reference. you need to find the index of the best and the second best point.
  //Initializecorrespondences
  c.clear();
  int last_best = -1;
  const int trans_size = trans_points.size();
  const int old_size = old_points.size();

  //Do for each point
  for (int ind_trans = 0; ind_trans < min(old_size, trans_size); ++ind_trans)
  {
    /// TODO: Implement Fast Correspondence Search

    int best = 0;
    int second_best = 0;
    double best_dist = std::numeric_limits<double>::infinity();
    int start_index = ind_trans;

    if(start_index<=0){start_index = 0;}
    else if(start_index >= old_size){start_index = old_size-1;}

    int we_start_at= (last_best!=-1) ? last_best+1 : start_index;

    int up = we_start_at+1;
    int down = we_start_at;

    double last_dist_up=10000;
    double last_dist_down=50000;

    bool up_stopped=false;
    bool down_stopped=false;

    double increment = 0.005823;

    while(!(up_stopped && down_stopped)){
      bool now_up = !up_stopped && (last_dist_up < last_dist_down);

      if(now_up){
        if(up>=old_size){up_stopped=true;continue;}
        last_dist_up = pow(trans_points[ind_trans].getX()-old_points[up].getX(),2) + pow(trans_points[ind_trans].getY()-old_points[up].getY(),2);

        if((last_dist_up < best_dist)){
          best = up;
          best_dist = last_dist_up;
        }

        if(up>start_index){
          double delta_up = (up-ind_trans)*increment;
          double min_distance_up = sin(delta_up)*trans_points[ind_trans].r;

          if ((min_distance_up*min_distance_up)>best_dist){up_stopped=true;continue;}
          if (old_points[up].r<trans_points[ind_trans].r){
            up=jump_table[up][1];
          }else{
            up=jump_table[up][0];
          }
        }else{up++;}
      }else{
        if(down<=-1){/*cout << "[10]stopped down";*/down_stopped=true;continue;}
        last_dist_down= pow(trans_points[ind_trans].getX()-old_points[down].getX(),2)+ pow(trans_points[ind_trans].getY()-old_points[down].getY(),2);
        if ((last_dist_down<best_dist)){
          best=down;
          best_dist=last_dist_down;
        }
        if (down<start_index){
          double delta_down = (up-ind_trans)*increment;
          double min_distance_down = sin(delta_down)*trans_points[ind_trans].r;
          if ((min_distance_down*min_distance_down)>best_dist){ /*cout << "[12]";*/down_stopped=true;continue;}
          if (old_points[down].r<trans_points[ind_trans].r){
            down=jump_table[down][3];
          }else{
            down=jump_table[down][2];
          }
        }else{
          down--;
        }
      }
    }

    last_best = best;
    if (best<=0){
      second_best=best+1;
    }else{
      second_best=best-1;
    }


    c.push_back(Correspondence(&trans_points[ind_trans], &points[ind_trans], &old_points[best], &old_points[second_best]));
  }
}

void computeJump(vector<vector<int>> &table, vector<Point> &points)
{
  table.clear();
  int n = points.size();
  for (int i = 0; i < n; ++i)
  {
    vector<int> v = {n, n, -1, -1};
    for (int j = i + 1; j < n; ++j)
    {
      if (points[j].r < points[i].r)
      {
        v[UP_SMALL] = j;
        break;
      }
    }
    for (int j = i + 1; j < n; ++j)
    {
      if (points[j].r > points[i].r)
      {
        v[UP_BIG] = j;
        break;
      }
    }
    for (int j = i - 1; j >= 0; --j)
    {
      if (points[j].r < points[i].r)
      {
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for (int j = i - 1; j >= 0; --j)
    {
      if (points[j].r > points[i].r)
      {
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}
