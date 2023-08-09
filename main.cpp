#include <vector>
#include <iostream>

#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "our_gl.h"

Model* model = NULL;
float* shadowbuffer = NULL;

const int width = 800;
const int height = 800;

Vec3f light_dir(1, 1, 1);
Vec3f       eye(1, 1, 3);
Vec3f    center(0, 0, 0);
Vec3f        up(0, 1, 0);

struct Shader : public IShader {
    mat<4, 4, float> uniform_M;         //  Projection*ModelView
    mat<4, 4, float> uniform_MIT;       // (Projection*ModelView).invert_transpose()
    mat<4, 4, float> uniform_Mshadow;   // 当前片段的屏幕坐标转换为阴影缓冲区内的屏幕坐标
    mat<2, 3, float> varying_uv;        // uv坐标齐次坐标, VS中写，FS中读
    mat<3, 3, float> varying_tri;       // 视口变换前的三角形齐次坐标，VS中写，FS中读

    Shader(Matrix M, Matrix MIT, Matrix MS) : uniform_M(M), uniform_MIT(MIT), uniform_Mshadow(MS), varying_uv(), varying_tri() {}

    virtual Vec4f vertex(int iface, int nthvert) {
        Vec4f gl_Vertex = ViewPort * Projection * ModelView * embed<4>(model->vert(iface, nthvert));    //变换矩阵
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));             //设置uv坐标
        varying_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));    //视口变换前的三角形齐次坐标
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor& color) {
        Vec4f sb_p = uniform_Mshadow * embed<4>(varying_tri * bar);     // corresponding point in the shadow buffer
        sb_p = sb_p / sb_p[3];
        int idx = int(sb_p[0]) + int(sb_p[1]) * width;                  // 深度检测数组下标
        float shadow = .3 + .7 * (shadowbuffer[idx] < sb_p[2] + 43.34); // 解决z-fighting问题 添加偏移量以消除相近平面影响
        Vec2f uv = varying_uv * bar;                                    // 对当前像素插值uv坐标
        Vec3f n = proj<3>(uniform_MIT * embed<4>(model->normal(uv))).normalize(); // 法线
        Vec3f l = proj<3>(uniform_M * embed<4>(light_dir)).normalize(); // 入射光线
        Vec3f r = (n * (n * l * 2.f) - l).normalize();                  // 反射光线
        float spec = pow(std::max(r.z, 0.0f), model->specular(uv));     // 高光
        float diff = std::max(0.f, n * l);                              // 漫反射
        TGAColor c = model->diffuse(uv);                                // 纹理
        //bling-phong模型
        //ambient + diffuse + specular
        //当深度测试通过时shadow为1对整体没有影响 未通过时乘偏移量0.3制造阴影
        for (int i = 0; i < 3; i++) color[i] = std::min<float>(20 + c[i] * shadow * (1.2 * diff + .6 * spec), 255);
        return false;
    }
};
//深度测试
struct DepthShader : public IShader {
    mat<3, 3, float> varying_tri;

    DepthShader() : varying_tri() {}

    virtual Vec4f vertex(int iface, int nthvert) {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert));            // read the vertex from .obj file
        gl_Vertex = ViewPort * Projection * ModelView * gl_Vertex;          // transform it to screen coordinates
        varying_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));    // setting triangle coordinates
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor& color) {
        Vec3f p = varying_tri * bar;                        //插值
        color = TGAColor(255, 255, 255) * (p.z / depth);    
        return false;
    }
};

int main(int argc, char** argv) {
    if (2 == argc) {
        model = new Model(argv[1]);
    }
    else {
        model = new Model("obj/african_head.obj");
    }
    //初始化深度数组
    float* zbuffer = new float[width * height];
    shadowbuffer = new float[width * height];
    for (int i = width * height; --i; ) {
        zbuffer[i] = shadowbuffer[i] = -std::numeric_limits<float>::max();
    }

    light_dir.normalize();

    {
        //深度测试渲染
        TGAImage depth(width, height, TGAImage::RGB);
        lookat(light_dir, center, up);
        viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
        projection(0);

        DepthShader depthshader;
        Vec4f screen_coords[3];
        for (int i = 0; i < model->nfaces(); i++) {
            for (int j = 0; j < 3; j++) {
                screen_coords[j] = depthshader.vertex(i, j);
            }
            triangle(screen_coords, depthshader, depth, shadowbuffer);
        }
        depth.flip_vertically(); // to place the origin in the bottom left corner of the image
        depth.write_tga_file("depth.tga");
    }

    Matrix M = ViewPort * Projection * ModelView;

    {
        // rendering
        TGAImage frame(width, height, TGAImage::RGB);
        lookat(eye, center, up);
        viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
        projection(-1.f / (eye - center).norm());
        //uniform_M uniform_MIT uniform_Mshadow
        Shader shader(ModelView, (Projection * ModelView).invert_transpose(), M * (ViewPort * Projection * ModelView).invert());
        Vec4f screen_coords[3];
        for (int i = 0; i < model->nfaces(); i++) {
            for (int j = 0; j < 3; j++) {
                screen_coords[j] = shader.vertex(i, j);
            }
            triangle(screen_coords, shader, frame, zbuffer);
        }
        frame.flip_vertically(); // to place the origin in the bottom left corner of the image
        frame.write_tga_file("framebuffer.tga");
    }

    delete model;
    delete[] zbuffer;
    delete[] shadowbuffer;

    return 0;
}