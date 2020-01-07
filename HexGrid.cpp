#include "HexGrid.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
using std::abs;
using std::max;


Vector3 axial_to_cube(Vector2 ax) {
  return Vector3(ax[0], ax[1], -ax[0] - ax[1]);
}

HexGrid::HexGrid(){
  origin = Vector2(0,0);

  hex_directions.push_back(Vector3(1,0,-1));
  hex_directions.push_back(Vector3(1,-1,0));
  hex_directions.push_back(Vector3(0,-1,1));
  hex_directions.push_back(Vector3(-1,0,1));
  hex_directions.push_back(Vector3(-1,1,0));
  hex_directions.push_back(Vector3(0,1,-1));

  setup_hex_transform();
 }

 void HexGrid::setup_hex_transform() {
  _base_hex_transform = Transform2D(
    1, 1.0/sqrt(3.0),
    0, 2.0/sqrt(3.0),
    origin.x, origin.y
    );
  set_hex_size(Vector2(50,50));
 }

void HexGrid::set_hex_size(Vector2 size_){
  size = size_;
  hex_transform = _base_hex_transform.scaled(size_);
  hex_transform_inv = hex_transform.affine_inverse();
}

void HexGrid::set_origin(Vector2 origin_){
  origin = origin_;
  setup_hex_transform();
}

Vector3 HexGrid::hex_add(Vector3 a, Vector3 b)
{
    return Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vector3 HexGrid::hex_subtract(Vector3 a, Vector3 b)
{
    return Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
}

Vector3 HexGrid::hex_scale(Vector3 a, int k)
{
    return Vector3(a.x * k, a.y * k, a.z * k);
}

Vector3 HexGrid::hex_direction(int direction)
{
    return hex_directions[direction];
}

Vector3 HexGrid::hex_neighbor(Vector3 hex, int direction)
{
    return hex_add(hex, hex_direction(direction));
}

Array HexGrid::hex_neighbors(Vector3 hex)
{
  Array results;
  for (int i = 0; i < 6; i++)
  {
      results.push_back(hex_neighbor(hex,i));
  }
    return results;
}


int HexGrid::hex_length(Vector3 hex)
{
    return int((abs(hex.x) + abs(hex.y) + abs(hex.z)) / 2);
}

int HexGrid::hex_distance(Vector3 a, Vector3 b)
{
    return hex_length(hex_subtract(a, b));
}

Vector3 HexGrid::hex_round(Vector3 h)
{
    int q = int(round(h.x));
    int r = int(round(h.y));
    int s = int(round(h.z));
    double q_diff = abs(q - h.x);
    double r_diff = abs(r - h.y);
    double s_diff = abs(s - h.z);
    if (q_diff > r_diff && q_diff > s_diff)
    {
        q = -r - s;
    }
    else
        if (r_diff > s_diff)
        {
            r = -q - s;
        }
        else
        {
            s = -q - r;
        }
    return Vector3(q, r, s);
}

Vector3 HexGrid::hex_lerp(Vector3 a, Vector3 b, double t)
{
    return Vector3(a.x * (1 - t) + b.x * t, a.y * (1 - t) + b.y * t, a.z * (1 - t) + b.z * t);
}

Array HexGrid::hex_linedraw(Vector3 a, Vector3 b)
{
    int N = hex_distance(a, b);
    Vector3 a_nudge = Vector3(a.x + 0.000001, a.y + 0.000001, a.z - 0.000002);
    Vector3 b_nudge = Vector3(b.x + 0.000001, b.y + 0.000001, b.z - 0.000002);
    Array results;
    double step = 1.0 / max(N, 1);
    for (int i = 0; i <= N; i++)
    {
        results.push_back(hex_round(hex_lerp(a_nudge, b_nudge, step * i)));
    }
    return results;
}

Vector2 HexGrid::hex_to_point(Vector3 h)
{
  return hex_transform.xform( Vector2(h.x, h.y) );
}

Vector3 HexGrid::point_to_hex(Vector2 p)
{
  Vector2 axial = hex_transform_inv.xform( p );
  return hex_round(axial_to_cube(axial));
}

// Array HexGrid::hex_edges(Vector3 hex){
// 	Array edges;
// 	Array corners = hex_corners(hex);
//   for (int i = 0; i < 6; i++){
// 		int l = i+1;
// 		if(l == 6){
// 			l = 0;
//     }
//     Array edge;
//     edge.push_back(corners[l]);
//     edge.push_back(corners[i]);
// 		edges.push_back(edge);
//   }
// 	return edges;
// }

//This makes a ring
Array HexGrid::hexes_at_distance(Vector3 hex,int dist)
{
  Array results;
  Vector3 pHex = hex_add(hex,hex_scale(hex_direction(4), dist));
	for (int i = 0; i < 6; i++)
    {
		    for (int j = 0; j < dist; j++){
          results.push_back(pHex);
          pHex = hex_neighbor(pHex,i);
        }
    }
	return results;
}

Array HexGrid::hexes_within_distance(Vector3 hex,int dist)
{
    Array results;
    results.push_back(hex);
    for (int i = 1; i < dist; i++){
      Array subR = hexes_at_distance(hex,i);
      for (int j = 0; j < subR.size(); j++){
        results.push_back(subR[j]);
      }
    }
    return results;
}

// bool HexGrid::los_clear_to(Vector3 start, Vector3 finish,Array obstacles){
//   bool clear = true;
//   Array line;
//   line.push_back(hex_to_point(start));
//   line.push_back(hex_to_point(finish));

// 	//This offcenters it just enough to not go through the point GHETTO i know but it sorta works enough
//   Vector2 line0 = line[0];
// 	line0.y = line0.y+0.005;

//   for(int i = 0; i < obstacles.size(); i++){
//     if(line.size() == 2 && obstacles.size() > i){
//       if(line_intersect_hex(obstacles[i],line0,line[1])){
//         clear = false;
//       }
//     }
//   }
//   return clear;
// }

// Array HexGrid::los_within_range(Vector3 hex,int dist, Array obstacles, Array checkList){
//   Array outList;
//   if(checkList.size() == 0){
//     checkList = hexes_within_distance(hex, dist);
//   }
//   for(int i = 0; i < checkList.size(); i++){
//     if(los_clear_to(checkList[i], hex,obstacles)){
//       outList.push_back(checkList[i]);
//     }
//   }
//   return outList;
// }

// bool HexGrid::lines_intersect(Vector2 l1Start,Vector2 l1End, Vector2 l2Start, Vector2 l2End){
//   bool results = false;

//   double line1StartX = l1Start.x;
// 	double line1StartY = l1Start.y;
// 	double line1EndX = l1End.x;
// 	double line1EndY = l1End.y;

// 	double line2StartX = l2Start.x;
// 	double line2StartY = l2Start.y;
// 	double line2EndX = l2End.x;
// 	double line2EndY = l2End.y;
//   double denominator = ((line2EndY - line2StartY) * (line1EndX - line1StartX)) - ((line2EndX - line2StartX) * (line1EndY - line1StartY));

//   if(denominator == 0){
//     return results;
//   }

//   double a = line1StartY - line2StartY;
// 	double b = line1StartX - line2StartX;
// 	double numerator1 = ((line2EndX - line2StartX) * a) - ((line2EndY - line2StartY) * b);
// 	double numerator2 = ((line1EndX - line1StartX) * a) - ((line1EndY - line1StartY) * b);
// 	a = numerator1 / denominator;
// 	b = numerator2 / denominator;

// 	if(a > 0 && a < 1 && b > 0 && b < 1){
// 		results = true;
//   }
// 	return results;
// }

// bool HexGrid::line_intersect_hex(Vector3 hex,Vector2 lStart, Vector2 lEnd){
// 	bool crosses = false;
// 	Array edges = hex_edges(hex);
// 	for (int i = 0; i < edges.size(); i++){
// 		if (edges.size() > i){
//       Array edge = edges[i];
// 			if (lines_intersect(lStart,lEnd,edge[0],edge[1])){
// 				crosses = true;
//       }
//     }
//   }
//   return crosses;
// }


// Array HexGrid::hexes_outlined(Array hexList){
// 	Array lines;

//   for(int i = 0; i < hexList.size(); i++){
//     Array hEdges = hex_edges(hexList[i]);
//     Array neighs =  hex_neighbors(hexList[i]);
//     neighs.invert();
//     int _i = 5;
//     while(_i >= 0){
//       if(hexList.find(neighs[_i]) > -1){
//         hEdges.remove(_i);
//       }
//       _i--;
//     }
//     for(int e = 0; e < hEdges.size(); e++){
//       lines.append(hEdges[e]);
//     }
//   }
//   return lines;
// }


Array HexGrid::get_directions() {
  return hex_directions;
}

Transform2D HexGrid::get_hex_transform(){
  return hex_transform;
}

Transform2D HexGrid::get_inv_transform(){
  return hex_transform_inv;
}

void HexGrid::_bind_methods() {

    ClassDB::bind_method("hex_add",&HexGrid::hex_add);
    ClassDB::bind_method("hex_subtract",&HexGrid::hex_subtract);
    ClassDB::bind_method("hex_scale",&HexGrid::hex_scale);
    ClassDB::bind_method("hex_distance",&HexGrid::hex_distance);
    ClassDB::bind_method("hex_neighbor",&HexGrid::hex_neighbor);
    ClassDB::bind_method("hex_neighbors",&HexGrid::hex_neighbors);
    ClassDB::bind_method("hex_round",&HexGrid::hex_round);
    ClassDB::bind_method("hex_linedraw",&HexGrid::hex_linedraw);
    ClassDB::bind_method("set_hex_size",&HexGrid::set_hex_size);
    ClassDB::bind_method("set_origin",&HexGrid::set_origin);

    ClassDB::bind_method("hex_to_point",&HexGrid::hex_to_point);
    ClassDB::bind_method("point_to_hex",&HexGrid::point_to_hex);
    // ClassDB::bind_method("hex_edges",&HexGrid::hex_edges);

    ClassDB::bind_method("get_hex_transform",&HexGrid::get_hex_transform);
    ClassDB::bind_method("get_inv_transform",&HexGrid::get_inv_transform);
    ClassDB::bind_method("get_directions",&HexGrid::get_directions);
    // ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM2D, "hex_transform"), 
      // "set_hex_transform", "get_hex_transform")

    // ClassDB::bind_method("line_intersect_hex",&HexGrid::line_intersect_hex);
    // ClassDB::bind_method("lines_intersect",&HexGrid::lines_intersect);
    // ClassDB::bind_method("los_clear_to",&HexGrid::los_clear_to);
    // ClassDB::bind_method("los_within_range",&HexGrid::los_within_range);

    ClassDB::bind_method("hexes_at_distance",&HexGrid::hexes_at_distance);
    ClassDB::bind_method("hexes_within_distance",&HexGrid::hexes_within_distance);
    // ClassDB::bind_method("hexes_outlined",&HexGrid::hexes_outlined);

}