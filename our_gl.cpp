#include <cmath>
#include <cstdlib>
#include <limits>
#include "our_gl.h"

Matrix ModelView;
Matrix ViewPort;
Matrix Projection;

IShader::~IShader() {}
//视口变换
void viewport(int x, int y, int w, int h) {
	ViewPort = Matrix::identity();
	ViewPort[0][3] = x + w / 2.f;
	ViewPort[1][3] = y + h / 2.f;
	ViewPort[2][3] = 255.f / 2.f;

	ViewPort[0][0] = w / 2.f;
	ViewPort[1][1] = h / 2.f;
	ViewPort[2][2] = 255.f / 2.f;
}
//投影变换
void projection(float coeff) {
	Projection = Matrix::identity();
	Projection[3][2] = coeff;
}
//视图变换
void lookat(Vec3f eye, Vec3f center, Vec3f up) {
	Vec3f z = (eye - center).normalize();
	Vec3f x = cross(up, z).normalize();
	Vec3f y = cross(z, x).normalize();
	ModelView = Matrix::identity();
	for (int i = 0; i < 3; i++) {
		ModelView[0][i] = x[i];
		ModelView[1][i] = y[i];
		ModelView[2][i] = z[i];
		ModelView[i][3] = -center[i];
	}
}
//重心坐标计算
Vec3f barycentric(Vec2f A, Vec2f B, Vec2f C, Vec2f P) {
	Vec3f s[2];
	for (int i = 2; i--; ) {
		s[i][0] = C[i] - A[i];
		s[i][1] = B[i] - A[i];
		s[i][2] = A[i] - P[i];
	}
	Vec3f u = cross(s[0], s[1]);
	if (std::abs(u[2]) > 1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
		return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
	return Vec3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}
//填充三角形
//输入 齐次坐标 着色器 图像 z-buffer
void triangle(Vec4f* pts, IShader& shader, TGAImage& image, float* zbuffer) {
	//bounding box
	Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 2; j++) {
			bboxmin[j] = std::min(bboxmin[j], pts[i][j] / pts[i][3]);
			bboxmax[j] = std::max(bboxmax[j], pts[i][j] / pts[i][3]);
		}
	}
	Vec2i P;
	TGAColor color;
	for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
		for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
			//重心坐标
			Vec3f c = barycentric(proj<2>(pts[0] / pts[0][3]), proj<2>(pts[1] / pts[1][3]), proj<2>(pts[2] / pts[2][3]), proj<2>(P));
			//深度插值
			float z = pts[0][2] * c.x + pts[1][2] * c.y + pts[2][2] * c.z;
			float w = pts[0][3] * c.x + pts[1][3] * c.y + pts[2][3] * c.z;
			int frag_depth = z / w;
			//不在三角形内或深度测试失败
			if (c.x < 0 || c.y < 0 || c.z<0 || zbuffer[P.x + P.y * image.get_width()]>frag_depth) continue;
			//片元着色器判断是否要丢弃该像素点（默认不丢弃）并 调用片元着色器
			bool discard = shader.fragment(c, color);
			if (!discard) {
				//更新深度数组并填充像素
				zbuffer[P.x + P.y * image.get_width()] = frag_depth;
				image.set(P.x, P.y, color);
			}
		}
	}
}