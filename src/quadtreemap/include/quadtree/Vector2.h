#ifndef Vector2_h_
#define Vector2_h_

#include <cmath>

struct Vector2;//声明要创建的数据结构
Vector2 operator*(int r, const Vector2& v);//声明要创建的函数，该函数在文件末尾，不在结构体内

struct Vector2 {
	/*union共用体，也叫联合体，在一个“联合”内可以定义多种不同的数据类型， 
	一个被说明为该“联合”类型的变量中，允许装入该“联合”所定义的任何一种数据，
	这些数据共享同一段内存，以达到节省空间的目的。
	 union变量所占用的内存长度等于最长的成员的内存长度。*/
	union {
		struct {
			// float x,y;
			int x,y;
		};
		// float D[2];
		int D[2];
	};

	Vector2() { } //初始化不赋值
	Vector2(int _x, int _y)//初始化并赋值
		:x(_x), y(_y)
	{ }

	int& operator[](unsigned int i) {//索引操作，会改变被索引对象
		return D[i];
	}

	const int& operator[](unsigned int i) const {//索引操作，不会改变被索引对象
		return D[i];
	}

	int maxComponent() const { //取最大值操作
		int r = x;
		if(y>r) r = y;
		return r;
	}

	int minComponent() const { //取最小值操作
		int r = x;
		if(y<r) r = y;
		return r;
	}

	Vector2 operator+(const Vector2& r) const { //元素相加操作
		return Vector2(x+r.x, y+r.y); 
	}

	Vector2 operator-(const Vector2& r) const { //元素相减操作
		return Vector2(x-r.x, y-r.y); 
	}

	Vector2 cmul(const Vector2& r) const { //元素相乘操作
		return Vector2(x*r.x, y*r.y);
	}

	Vector2 cdiv(const Vector2& r) const { //元素相除操作
		return Vector2(x/r.x, y/r.y);
	}

	Vector2 operator*(int r) const { //乘法操作
		return Vector2(x*r,y*r);
	}

	Vector2 operator/(int r) const { //除法操作
		return Vector2(x/r, y/r);
	}

	Vector2& operator+=(const Vector2& r) { //指针-元素相加操作
		x+=r.x;
		y+=r.y;
		return *this;
	}

	Vector2& operator-=(const Vector2& r) { //指针-元素相减操作
		x-=r.x;
		y-=r.y;
		return *this;
	}

	Vector2& operator*=(int r) { //指针-乘法操作
		x*=r; y*=r;
		return *this;
	}

	// Inner/dot product
	int operator*(const Vector2& r) const { //对应元素平方和操作
		return x*r.x + y*r.y ;
	}

	int norm() const {//对应元素平方和后开方操作
		return sqrtf(x*x+y*y);
	}

	int normSquared() const { //元素平方和操作
		return x*x + y*y ;
	}

	Vector2 normalized() const { //归一化操作
		return *this / norm();
	}
};

inline Vector2 operator*(int r, const Vector2& v) { //结构体数据乘以一个数值
	return Vector2(v.x*r, v.y*r);
}

#endif
