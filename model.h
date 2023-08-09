#ifndef __MODEL_H__
#define __MODEL_H__
#include <vector>
#include <string>
#include "geometry.h"
#include "tgaimage.h"

class Model {
private:
	std::vector<Vec3f> verts_;		//����
	std::vector<std::vector<Vec3i> > faces_;	// vertex/uv/normal index
	std::vector<Vec3f> norms_;		//����
	std::vector<Vec2f> uv_;			//����uv����
	//Bling-Phong
	TGAImage diffusemap_;			//����ͼ��Ҳ����diffuse��
	TGAImage normalmap_;			//������ͼ
	TGAImage specularmap_;			//�߹�ͼ�����ڲ��߹⣩
	//���ز���
	void load_texture(std::string filename, const char* suffixl, TGAImage& img);
public:
	Model(const char* filename);
	~Model();
	int nverts();		//��͵��������
	int nfaces();
	Vec3f normal(int iface, int nthvert);	//���ط�������
	Vec3f normal(Vec2f uv);
	Vec3f vert(int i);						//����ָ��������
	Vec3f vert(int iface, int nthvert);		
	Vec2f uv(int iface, int nthvert);		//����uv����
	TGAColor diffuse(Vec2f uv);				//����
	float specular(Vec2f uv);				//�߹�
	std::vector<int> face(int idx);			//���������ε�������������
};

#endif // !__MODEL_H__