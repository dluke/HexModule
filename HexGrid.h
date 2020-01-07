#ifndef HEXGRID_H
#define HEXGRID_H
#include "scene/main/node.h"
#include "core/math/transform_2d.h"

enum DIR {NE, SE, S, SW, NW, N};

class HexGrid : public Node {
	GDCLASS(HexGrid,Node);
	Vector2 size;
	Vector2 origin;
	Array hex_directions;
	Transform2D _base_hex_transform;
	Transform2D hex_transform;
	Transform2D hex_transform_inv;

protected:
	void setup_hex_transform();
	static void _bind_methods();

public:

	Vector3 hex_add(Vector3 a, Vector3 b);
	Vector3 hex_subtract(Vector3 a, Vector3 b);
	Vector3 hex_scale(Vector3 a, int k);
	Vector3 hex_direction(int direction);
	Vector3 hex_neighbor(Vector3 hex, int direction);
	Array hex_neighbors(Vector3 hex);
	int hex_length(Vector3 hex);
	int hex_distance(Vector3 a, Vector3 b);
	Vector3 hex_round(Vector3 h);
	Vector3 hex_lerp(Vector3 a, Vector3 b, double t);
	Array hex_linedraw(Vector3 a, Vector3 b);
	Vector2 hex_to_point(Vector3 h);
	Vector3 point_to_hex(Vector2 p);
	// Array hex_edges(Vector3 hex);

	Array hexes_at_distance(Vector3 hex,int dist);
	Array hexes_within_distance(Vector3 hex,int dist);
	// Array hexes_outlined(Array hexList);

	// bool line_intersect_hex(Vector3 hex,Vector2 lStart, Vector2 lEnd);
	// bool lines_intersect(Vector2 l1Start,Vector2 l1End, Vector2 l2Start, Vector2 l2End);
	// bool los_clear_to(Vector3 start, Vector3 finish,Array obstacles);
	// Array los_within_range(Vector3 hex,int dist, Array obstacles, Array checkList);

	void set_hex_size(Vector2 size_);
	void set_origin(Vector2 origin_);
	Transform2D get_hex_transform();
	Transform2D get_inv_transform();
	Array get_directions();

	HexGrid();
};

#endif
