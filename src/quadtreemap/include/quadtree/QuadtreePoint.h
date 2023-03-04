#ifndef QuadtreePoint_H
#define QuadtreePoint_H

#include "Vector2.h"

// Simple point data type to insert into the tree.
// Have something with more interesting behavior inherit  from this in order to store other attributes in the tree.
// 要插入八叉树中简单点的数据类型。
// 继承一些具有更有趣行为的东西，以便在树中存储其他属性。
class QuadtreePoint {
	Vector2 position; 
public:
	QuadtreePoint() { }//初始化OctreePoint
	QuadtreePoint(const Vector2& position) : position(position) { } //初始化OctreePoint，并给OctreePoint的position赋值
	inline const Vector2& getPosition() const { return position; }  //返回/获取位置数据
	inline void setPosition(const Vector2& p) { position = p; }        //设置位置数据
};

#endif
