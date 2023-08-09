#ifndef __OUR_GL_H__
#define __OUR_GL_H__

#include "tgaimage.h"
#include "geometry.h"

extern Matrix ModelView;
extern Matrix ViewPort;
extern Matrix Projection;
const float depth = 2000.f;

void viewport(int x, int y, int w, int h);
void projection(float coeff = 0.f);
void lookat(Vec3f eye, Vec3f center, Vec3f up);

//抽象类 
struct IShader {
	virtual ~IShader();
	//纯虚函数 顶点着色器 和 片元着色器必须实现
	virtual Vec4f vertex(int iface, int nthvert) = 0;
	virtual bool fragment(Vec3f bar, TGAColor& color) = 0;
};

void triangle(Vec4f* pts, IShader& shader, TGAImage& image, float* zbuffer);

#endif // !__OUR_GL_H__
