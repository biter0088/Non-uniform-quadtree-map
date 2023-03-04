#ifndef Quadtree_H
#define Quadtree_H

#include <cstddef>
#include <vector>
#include "QuadtreePoint.h"

namespace brandonpelfrey {

	/**!
	 *定义一个八叉树的类，其组成包括:
	 	当前节点中心位置origin、当前节点对应空间尺寸halfDimension、
	 	当前节点的子节点children、当前节点的数据data
	 */
	class Quadtree {
		// Physical position/size. This implicitly defines the bounding box of this node   
		// 物体位置和大小，间接定义了当前节点框边界
		public: 
  		Vector2 origin;         //! The physical center of this node  该节点的物理中心
		Vector2 halfDimension;  //! Half the width/height/depth of this node 该节点的半宽/高/深
		int depth;//四叉树节点的深度
		int type;//四叉树节点的占据类型，0空闲，1占据，-1未知。    ps:这里先用比较简单的占据类型来处理占据概率问题

		//这棵树最多有八个子节点，分别可以额外存储一个点，不过在许多应用程序中，叶子将存储数据。
		// The tree has up to eight children and can additionally store 
		// a point, though in many applications only, the leaves will store data.
		Quadtree *children_root[100]; //根节点的子节点，因为使用非均匀四叉树地图，所以根节点的子节点更多
		Quadtree *children[4]; //! Pointers to child octants   指向八叉树子节点的指针
		QuadtreePoint *data;   //! Data point to be stored at a node  一个指针--指向存储在一个八叉树节点中的数据

		/*
				Children follow a predictable pattern to make accesses simple.
				八叉树子节点遵循可预测的模式以使访问变得简单。
				Here, - means less than 'origin' in that dimension, + means greater than.
				这里，- 表示小于该维度(坐标)的“原点”，+ 表示大于。

				child:	0 1 2 3  //从下面xy的+-表示中可以看出这个4个子节点的空间分布
				----需要与地图坐标系进行匹配
				----假设地图坐标系为速腾雷达坐标系，x前y左
				x:      -  -  + + 
				y:      -  + -  + 
				------------
				3 --+-- 2
				1 --+-- 0
				----+------

				ros地图坐标系: x右y上：
			    ------------
				// 1--+-- 0
				// 3 --+-- 2
				// ----+------


		 */

		//public:  hxz
		Quadtree(const Vector2& origin, const Vector2& halfDimension) //初始化一个四叉树
			: origin(origin), halfDimension(halfDimension), data(NULL) {
				// Initially, there are no children
				for(int i=0; i<4; ++i) 
					children[i] = NULL;
				for(int i=0; i<100; ++i) 
					children_root[i] = NULL;
			}

		Quadtree(const Vector2& origin, const Vector2& halfDimension,const int& depth) //初始化一个四叉树，带有深度信息
			: origin(origin), halfDimension(halfDimension), depth(depth), data(NULL) {
				// Initially, there are no children
				for(int i=0; i<4; ++i) 
					children[i] = NULL;
				for(int i=0; i<100; ++i) 
					children_root[i] = NULL;
			}

		Quadtree(const Vector2& origin, const Vector2& halfDimension,const int& depth,const bool& root_ornot) //初始化一个四叉树，带有深度信息
			: origin(origin), halfDimension(halfDimension), depth(depth), data(NULL) {
				// Initially, there are no children
				for(int i=0; i<4; ++i) 
					children_root[i] = NULL;
				for(int i=0; i<100; ++i) 
					children_root[i] = NULL;
			}


		Quadtree(const Quadtree& copy) //从另一个八叉数复制得到一个八叉树
			: origin(copy.origin), halfDimension(copy.halfDimension), data(copy.data) {

			}

		~Quadtree() {
			// Recursively destroy octants //递归删除子节点
			for(int i=0; i<4; ++i) 
				delete children[i];
		}

		// 确定point在该八叉树的哪一个子节点空间内
		// Determine which octant of the tree would contain 'point'
		int getOctantContainingPoint(const Vector2& point) const {
			int oct = 0;
			if(point.x >= origin.x){
				oct=2;
				if(point.y >= origin.y){
					oct=3;
				}
			}
			if(point.x <= origin.x&&point.y >= origin.y) oct = 1; 
			return oct;//这个值应该是0、1、2、3之中的一个
		}

		//检查是否是叶子节点(叶子节点--没有子节点的八叉树节点)
		bool isLeafNode() const {
			// This is correct, but overkill--这是正确的，但矫枉过正. See below.
			/*
				 for(int i=0; i<4; ++i)
				 if(children[i] != NULL) 
				 return false;
				 return true;
			 */

			// We are a leaf if we have no children. Since we either have none, or  all eight, it is sufficient to just check the first.
			// 如果没有子节点，就是一片叶子节点。 由于要么没有，要么全部都没有，所以只检查第一个就足够了。
			return children[0] == NULL;
		}

		//插入一个简单数据点，即将简单数据点插入到八叉树中
		void insert(QuadtreePoint* point) {
			// If this node doesn't have a data point yet assigned  and it is a leaf, then we're done!
			// 如果八叉树当前节点还没有分配一个数据点并且它是一个叶子节点，那么我们就完成了----把该简单数据点赋值到当前节点即可
			if(isLeafNode()) { //是否是叶子节点----没有子节点
				if(data==NULL) { //当是叶子节点并且没有被赋值时，将简单数据点赋值给它
					data = point;
					return;
				} else { 
					// We're at a leaf, but there's already something here
					// We will split this node so that it has 8 child octants and then insert the old data that was here, along with this new data point 
					// Save this data point that was here for a later re-insert
					// 是叶子节点并且已经被赋值时
					// 将分割这个叶子节点，使其有 8 个子节点，然后插入这里的旧数据以及这个新数据点
					// 保存此数据点，以便稍后重新插入
					QuadtreePoint *oldPoint = data; //保存叶子节点的数据
					data = NULL;

					// Split the current node and create new empty trees for each  child octant.
					// 拆分当前叶子节点，并为每个子节点创建新的空八叉树。
					for(int i=0; i<4; ++i) {
						// Compute new bounding box for this child 为子节点创建新的边界框
						Vector2 newOrigin = origin;
						//&运算符、三目运算符
						newOrigin.x += halfDimension.x * (i>1 ? .5f : -.5f);//i为2、3时为+，0、1时为-
						newOrigin.y += halfDimension.y * (i&1 ? .5f : -.5f);//i为1、3时为+，0、2时为-
						children[i] = new Quadtree(newOrigin, halfDimension*.5f);
					}

					// Re-insert the old point, and insert this new point 重新插入旧点，并插入这个新点
					// (We wouldn't need to insert from the root, because we already know it's guaranteed to be in this section of the tree)
					//（我们不需要从根插入，因为我们已经知道它保证在八叉树的某一个子节点对应的空间内）
					// 这里对子节点进行插入“简单数据点”操作，观察children[]里面得到的子节点的序号
					children[getOctantContainingPoint(oldPoint->getPosition())]->insert(oldPoint);
					children[getOctantContainingPoint(point->getPosition())]->insert(point);
				}
			} else {
				// We are at an interior node. Insert recursively into the  appropriate child octant
				// 当八叉树有子节点时，根据需要插入的简单数据点得到子节点的序号，并将该简单数据点插入其中
				int octant = getOctantContainingPoint(point->getPosition());
				children[octant]->insert(point);
			}
		}

		// This is a really simple routine for querying the tree for points within a bounding box defined by min/max points (bmin, bmax)
		// 这是一个非常简单的例程，用于查询八叉树中（bmin，bmax）定义的边界框内的八叉树节点
		// All results are pushed into 'results'
		void getPointsInsideBox(const Vector2& bmin, const Vector2& bmax, std::vector<QuadtreePoint*>& results) {
			// If we're at a leaf node, just see if the current data point is inside the query bounding box
			// 如果我们在叶子节点，只需查看当前叶子节点是否在查询边界框内
			if(isLeafNode()) {
				if(data!=NULL) {
					const Vector2& p = data->getPosition();//当前叶子节点的数据
					if(p.x>bmax.x || p.y>bmax.y ) return;
					if(p.x<bmin.x  || p.y<bmin.y  ) return;
					results.push_back(data);
				}
			} else {
				// We're at an interior node of the tree. We will check to see if  the query bounding box lies outside the octants of this node.
				// 对于八叉树的内部节点----非叶子节点即有子节点的节点。 我们将检查这八个子节点是否位于查询边界框内
				for(int i=0; i<4; ++i) {
					// Compute the min/max corners of this child octant 计算这个某个子节点的最小/最大范围
					Vector2 cmax = children[i]->origin + children[i]->halfDimension;
					Vector2 cmin = children[i]->origin - children[i]->halfDimension;

					// If the query rectangle is outside the child's bounding box,  then continue
					// 如果查询矩形在子节点的边界框之外，则继续（其他子节点或向上一级节点搜索）
					if(cmax.x<bmin.x || cmax.y<bmin.y ) continue;
					if(cmin.x>bmax.x || cmin.y>bmax.y ) continue;

					// At this point, we've determined that this child is intersecting  the query bounding box
					// 至此，我们已经确定这个子节点在查询边界框内，进一步进行搜索(这个子节点是否有子子节点。。。。)
					children[i]->getPointsInsideBox(bmin,bmax,results);
				} 
			}
		}

	};

}
#endif
