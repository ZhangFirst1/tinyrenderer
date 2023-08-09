#ifndef __MODEL_H__
#define __MODEL_H__
#include <vector>
#include <string>
#include "geometry.h"
#include "tgaimage.h"

class Model {
private:
	std::vector<Vec3f> verts_;		//顶点
	std::vector<std::vector<Vec3i> > faces_;	// vertex/uv/normal index
	std::vector<Vec3f> norms_;		//法线
	std::vector<Vec2f> uv_;			//纹理uv坐标
	//Bling-Phong
	TGAImage diffusemap_;			//纹理图（也就是diffuse）
	TGAImage normalmap_;			//法线贴图
	TGAImage specularmap_;			//高光图（用于补高光）
	//加载材质
	void load_texture(std::string filename, const char* suffixl, TGAImage& img);
public:
	Model(const char* filename);
	~Model();
	int nverts();		//面和点的总数量
	int nfaces();
	Vec3f normal(int iface, int nthvert);	//返回方向向量
	Vec3f normal(Vec2f uv);
	Vec3f vert(int i);						//返回指定点坐标
	Vec3f vert(int iface, int nthvert);		
	Vec2f uv(int iface, int nthvert);		//返回uv坐标
	TGAColor diffuse(Vec2f uv);				//纹理
	float specular(Vec2f uv);				//高光
	std::vector<int> face(int idx);			//返回三角形的三个顶点坐标
};

#endif // !__MODEL_H__