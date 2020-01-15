#include <iostream>
#include <thread>
#include "xmlload.cpp"
#include "viewport.cpp"
#include "texture.cpp"
#include "objects.h"
using namespace std;

Node rootNode;
Camera camera;
RenderImage renderImage;
Sphere theSphere;
Plane thePlane;
ObjFileList objList;
MaterialList materials;
LightList lights;
TexturedColor background;
TexturedColor environment;
TextureList textureList;

class pixelIterator
{
private:
    atomic<int> n;
public:
    void Init(){ n = 0;}
    bool PixelID(int &i, int &j, int &V0){
        V0 = n++;
        if( V0 >= camera.imgWidth*camera.imgHeight ) return false;
        i = V0%camera.imgWidth;
        j = V0/camera.imgWidth;
        return true;
    }
};
pixelIterator pID;

bool TraceShadowRay(const Node &node,const Ray &ray, HitInfo &hitinfo, float t_max){
    //transform camera ray from world space to object space
    float bias = 1e-6f;
    Ray r = node.ToNodeCoords(ray);
    const Object *obj = node.GetNodeObj();
    if(obj){
        if(obj->IntersectRay(r,hitinfo)){
            if(hitinfo.z>bias && hitinfo.z <= t_max){
                return true;
            }
        }
    }
    for(int i=0; i<node.GetNumChild(); i++){
        const Node &child = *node.GetChild(i);
        if(TraceShadowRay(child, r, hitinfo, t_max)){
            return true;
        }
    }
    return false;
}

float GenLight::Shadow(Ray ray, float t_max)
{
    HitInfo hitInfo;
    hitInfo.Init();
    return not TraceShadowRay(rootNode,ray,hitInfo,t_max);
}

Color PointLight::Illuminate(Vec3f const &p, Vec3f const &N) const
{   
    if(size>0){
        // compute coordinate axes
        Vec3f Z = (position-p).GetNormalized();
        Vec3f R = Vec3f(0,0,1);
        if(Z.z>0.8) R = Vec3f(0,1,0);
        Vec3f X = R ^ Z;
        Vec3f Y = X ^ Z;
        X.Normalize();
        Y.Normalize();
        Matrix3f CA(X, Y, Z);
        // declare camera, pixel variables
        Vec3f lightpos, dir;
        float r, theta, s=rand()%100;    
        int min_sample = 4;
        int max_sample = 4;  
        int num_sample = 0;
        float shadow = 0;
        while(num_sample < max_sample){
            // generate light position sample
            r = size*sqrt(((float)rand())/RAND_MAX);
            theta = 2*M_PI*((float)rand())/RAND_MAX;
            lightpos = CA*Vec3f(r*cos(theta), r*sin(theta), 0);        
            // trace the shadow ray
            shadow += Shadow(Ray(p, position+lightpos-p),1);
            num_sample++;
            if(num_sample==min_sample && shadow!=num_sample && shadow!=0) break;
        }
        return shadow/(float)num_sample*intensity;    
    } 
    else return Shadow(Ray(p,position-p),1) * intensity;
}
	
bool Sphere::IntersectRay( const Ray &ray, HitInfo &hitinfo, int hitSide) const
{
	float V0 = ray.dir.Dot(ray.dir);
	float V1 = 2*ray.p.Dot(ray.dir);
	float V2 = ray.p.Dot(ray.p)-1;
	float delta = V1*V1-(4*V0*V2);
    float zero = 1e-3f;
	
	if(delta>=zero){
		float t1 = (-V1-sqrtf(delta))/(2*V0);
		float t2 = (-V1+sqrtf(delta))/(2*V0);
		
        if(t1>zero && t2 < BIGFLOAT && t1<hitinfo.z){
        	hitinfo.z = t1;
            hitinfo.p = ray.p + hitinfo.z*ray.dir;
            hitinfo.N = hitinfo.p.GetNormalized();
            float u = 0.5 - atan2(hitinfo.p.x,hitinfo.p.y)/ (2 * M_PI);
            float v = 0.5 + asin(hitinfo.p.z)/M_PI;
            hitinfo.uvw = Vec3f(u,v,0);
            hitinfo.front = true;
        	return true;
        }
        if(t2>zero && t2 < BIGFLOAT && t2<hitinfo.z){
        	hitinfo.z = t2;
            hitinfo.p = ray.p + hitinfo.z*ray.dir;
            hitinfo.N = hitinfo.p.GetNormalized();
            float u = 0.5 - atan2(hitinfo.p.x,hitinfo.p.y)/ (2 * M_PI);
            float v = 0.5 + asin(hitinfo.p.z)/M_PI;
            hitinfo.uvw = Vec3f(u,v,0);
            hitinfo.front = false;
        	return true;
        }
	}
	return false;
}

bool Plane::IntersectRay( const Ray &ray, HitInfo &hitinfo, int hitSide) const{
    float zero = 1e-3f; 
    float t = -(ray.p.z/ray.dir.z);

    if(t > zero && t < BIGFLOAT && t<hitinfo.z){
        Vec3f X = ray.p + t*ray.dir;
        if(X.x >= -1 && X.x <= 1 && X.y >=-1 && X.y <=1){
            hitinfo.z = t;
            hitinfo.p = X;
            hitinfo.N = Vec3f(0,0,1);
            float u = 0.5 + hitinfo.p.x/2;
            float v = 0.5 + hitinfo.p.y/2;
            hitinfo.uvw = Vec3f(u,v,0);
            if(ray.dir.z < 0.0){
                hitinfo.front = true;
            }
            else hitinfo.front = false;
            return true;
        }
    }
    return false;
}

bool TriObj::IntersectTriangle( Ray const &ray, HitInfo &hInfo, int hitSide, unsigned int faceID ) const{
    float bias = 1e-5f;
    TriFace face = F(faceID);
    Vec3f V0 = V(face.v[0]);
    Vec3f V1 = V(face.v[1]);
    Vec3f V2 = V(face.v[2]);
    Vec3f N = (V1 - V0).Cross(V2 - V0);
    N.Normalize();
    if(N.Dot(ray.p-V0) < bias) return false;
    if(abs(N.Dot(ray.dir)) < bias) return false;
    float t = N.Dot(V2-ray.p)/N.Dot(ray.dir);
    
    if(t<bias || t>=hInfo.z || t>=BIGFLOAT) return false;
    Vec3f X = ray.p + t * ray.dir;
    
    N.x = abs(N.x);    
    N.y = abs(N.y);    
    N.z = abs(N.z);
    float maxN = max(max(N.x,N.y),N.z);
    
    Vec2f P0, P1, P2, PX;
    if(maxN==N.x){
        P0 = Vec2f(V0.y, V0.z);
        P1 = Vec2f(V1.y, V1.z);
        P2 = Vec2f(V2.y, V2.z);
        PX = Vec2f(X.y, X.z);
    }
    else if(maxN==N.y){
        P0 = Vec2f(V0.x, V0.z);
        P1 = Vec2f(V1.x, V1.z);
        P2 = Vec2f(V2.x, V2.z);
        PX = Vec2f(X.x, X.z);
    }
    else{
        P0 = Vec2f(V0.x, V0.y);
        P1 = Vec2f(V1.x, V1.y);
        P2 = Vec2f(V2.x, V2.y);
        PX = Vec2f(X.x, X.y);
    }
    
    float A = (P1 - P0).Cross(P2 - P0);
    float A0 = (P1 - PX).Cross(P2 - PX)/A;
    float A1 = (P2 - PX).Cross(P0 - PX)/A;
    float A2 = 1.0 - A0 - A1;
    if(A0<bias || A1<bias || A2<bias) return false;
    
    hInfo.z = t;
    hInfo.p = A0 * V0 + A1 * V1 + A2 * V2;
    hInfo.N = GetNormal(faceID, Vec3f(A0,A1,A2));
    hInfo.N.Normalize();
    hInfo.uvw = GetTexCoord(faceID,Vec3f(A0,A1,A2));
    hInfo.front = true;
    
    return true;
}

bool TriObj::IntersectRay( const Ray &ray, HitInfo &hitinfo, int hitSide) const{
    return TraceBVHNode(ray, hitinfo, hitSide, bvh.GetRootNodeID());
}

float Box::IntersectRay(const Ray &r, float t_max) const{
        
    if (IsInside(r.p)) return true;
    
    Vec3f b0 = Corner(0);
    Vec3f b1 = Corner(7);
    float t;

    float t0 = (b0.x-r.p.x)/r.dir.x;
    float t1 = (b1.x-r.p.x)/r.dir.x;
    if(t0>t1){
        t = t0;
        t0 = t1;
        t1 = t;
    }
    
    float ty0 = (b0.y-r.p.y)/r.dir.y;
    float ty1 = (b1.y-r.p.y)/r.dir.y;
    if(ty0>ty1){
        t = ty0;
        ty0 = ty1;
        ty1 = t;
    }

    if ((t0 > ty1) || (ty0 > t1)) return false; 
    if (ty0 > t0) t0 = ty0; 
    if (ty1 < t1) t1 = ty1; 
    
    float tz0 = (b0.z-r.p.z)/r.dir.z;
    float tz1 = (b1.z-r.p.z)/r.dir.z;
    if(tz0>tz1){
        t = tz0;
        tz0 = tz1;
        tz1 = t;
    }
    
    if ((t0 > tz1) || (tz0 > t1)) return false; 
    if (tz0 > t0) t0 = max(t0,tz0); 
    if (tz1 < t0) t1 = max(t1,tz1);
    
    if(t0<=t1 && t1<=t_max) return t0;
    return 0.0f;
}

bool TriObj::TraceBVHNode(const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int nodeID) const{        
    Box root_box = Box(bvh.GetNodeBounds(nodeID));
    bool hit = false;
    
    if(root_box.IntersectRay(ray, BIGFLOAT)>0){
        if(bvh.IsLeafNode(nodeID)){
            unsigned int* elements = (unsigned int*) bvh.GetNodeElements(nodeID);
            for(int i=0; i<bvh.GetNodeElementCount(nodeID); i++){
                if(IntersectTriangle(ray, hInfo, hitSide, elements[i])) hit = true;
            }
        }
        else{
            unsigned int child1, child2;
            bvh.GetChildNodes(nodeID, child1, child2);
            Box box1 = Box(bvh.GetNodeBounds(child1));
            Box box2 = Box(bvh.GetNodeBounds(child2));
            float t1 = box1.IntersectRay(ray, BIGFLOAT);
            float t2 = box2.IntersectRay(ray, BIGFLOAT);
            
            if(t1>0 && t2>0){
                if(t1<=t2){
                    if(t1<hInfo.z){
                        if(TraceBVHNode(ray, hInfo, hitSide, child1)) hit = true;
                        if(t2<hInfo.z){
                            if(TraceBVHNode(ray, hInfo, hitSide, child2)) hit = true;    
                        }    
                    }
                }
                else{
                    if(t2<hInfo.z){
                        if(TraceBVHNode(ray, hInfo, hitSide, child2)) hit = true;
                        if(t1<hInfo.z){
                            if(TraceBVHNode(ray, hInfo, hitSide, child1)) hit = true;    
                        }    
                    }
                }
            }
            else if(t1>0 && t1<hInfo.z){
                if(TraceBVHNode(ray, hInfo, hitSide, child1)) hit = true;
            }
            else if(t2>0 && t2<hInfo.z){
                if(TraceBVHNode(ray, hInfo, hitSide, child2)) hit = true;
            }
        }
    }
    return hit;
}

bool TraceNode(const Node &node,const Ray &ray, HitInfo &hitinfo){
    //transform camera ray from world space to object space
    Ray r = node.ToNodeCoords(ray);
    bool hit = false;
    const Object *obj = node.GetNodeObj();
    if(obj){
        if(obj->IntersectRay(r,hitinfo)){
            hit = true;
            hitinfo.node = &node;
            node.FromNodeCoords(hitinfo);
        }
    }
    for(int i=0; i<node.GetNumChild(); i++){
        const Node &child = *node.GetChild(i);
        if(TraceNode(child, r, hitinfo)){
            hit = true;
            node.FromNodeCoords(hitinfo);
        }
    }
    return hit;
}

bool TraceRay(const Ray &ray, HitInfo &hitinfo){
    return TraceNode(rootNode, ray, hitinfo);
}

void updatePixel(int n, float z, int s, Color color){
	// update pixel color value
    float gamma = 2.2;
    color.r = pow(color.r, (1.0 / gamma));
    color.g = pow(color.g, (1.0 / gamma));
    color.b = pow(color.b, (1.0 / gamma));
    Color24* pColor = renderImage.GetPixels();
    pColor[n] = (Color24) color;
    // update pixel z value
    float* zBuffer = renderImage.GetZBuffer();
    zBuffer[n] = z;
    // update pixel sample count
    uint8_t* sampleCount = renderImage.GetSampleCount();
    sampleCount[n] = (uint8_t) s; 
}

Vec3f GlossyNormal(Vec3f normal, float radius)
{
    Vec3f Z = normal;
    Vec3f R = Vec3f(0,0,1);
    if(Z.z>0.8) R = Vec3f(0,1,0);
    Vec3f X = R ^ Z;
    Vec3f Y = X ^ Z;
    X.Normalize();
    Y.Normalize();
    Matrix3f CA(X, Y, Z);
    float r = radius*sqrt(((float)rand())/RAND_MAX);
    float theta = 2*M_PI*((float)rand())/RAND_MAX;
    return (normal+CA*Vec3f(r*cos(theta), r*sin(theta), 0)).GetNormalized();
}

Color MtlBlinn::Shade(Ray const &ray, const HitInfo &hInfo, const LightList &lights, int bounceCount) const
{
    Color output(0.0);
    Vec3f V = (-ray.dir).GetNormalized();
    Vec3f P = hInfo.p;
    const Material *mat = hInfo.node->GetMaterial();
    const MtlBlinn *mtl = static_cast<const MtlBlinn*>(mat);
    Color Kd = mtl->diffuse.Sample(hInfo.uvw, hInfo.duvw);
    Color Ks = mtl->specular.Sample(hInfo.uvw, hInfo.duvw);
    float alpha = mtl->glossiness;
    
    Color LI;
    Vec3f N, L, H, RR;
    float diffuse, specular;
    // direct illumination
    N = hInfo.N;
    for(int i=0; i<lights.size(); i++){
        // ambient light
        LI = lights[i]->Illuminate(P,N); 
        if(lights[i]->IsAmbient()){
            output += LI * Kd;
        }
        else{
            // diffuse reflection
            L = -1*lights[i]->Direction(P);
            diffuse = N.Dot(L);
            if(diffuse>0){
                output += LI * diffuse * Kd;
                // phong
                // RR = (N*2.0*L.Dot(N)-L).GetNormalized();
                // specular = V.Dot(RR);
                // blinn phong
                H = (L+V).GetNormalized();
                specular = N.Dot(H);
                // specular reflection
                if(specular>0)
                    output += LI * pow(specular,alpha) * Ks;
            }
        }
    }

    float R=0;
    // refraction
    if(mtl->refraction.GetColor().Sum()>0 && bounceCount>0){
        if (refractionGlossiness > 0) N = GlossyNormal(hInfo.N,refractionGlossiness);
        else N = hInfo.N;
        float n;    // n1/n2 ratio
        if(hInfo.front){
            n = 1/mtl->ior;
        }
        else{
            n = mtl->ior;
            N = -N;
        }

        float cos1 = V.Dot(N);
        float sinsq1 = 1 - cos1*cos1;
        float sin1 = sqrtf(sinsq1);
        float sin2 = n * sin1;
        
        if(sin2<1.0){
            float cos2 = sqrtf(max(0.0,1.0 - n*n*sinsq1));
            // initialize hitinfo
            HitInfo hitinfo;
            hitinfo.Init();
            Vec3f refracted_dir = -cos2*N -sin2*((V-cos1*N)).GetNormalized();
            Ray refracted_ray(P, refracted_dir);
            Color refracted_color(0.0);
            if(TraceRay(refracted_ray, hitinfo)){
                refracted_color = hitinfo.node->GetMaterial()->Shade(refracted_ray,hitinfo,lights,bounceCount-1);
            }
            else{
                refracted_color = environment.SampleEnvironment(refracted_ray.dir);
            }
            // absorption
            if(!hitinfo.front){
                Color a = mtl->absorption;
                float t = hitinfo.z;
                refracted_color *= Color(exp(-a.r*t), exp(-a.g*t), exp(-a.b*t));               
            }
            // Schlick's Approximation
            float R0 = pow((n-1)/(n+1), 2);
            R = R0 + (1-R0)*pow(1-cos1,5);
            output += (1-R)*mtl->refraction.GetColor()*refracted_color;
        }
        // total internal reflection
        else{
            R = 1;
        }
    }

    // reflection
    if((mtl->reflection.GetColor().Sum()>0 || R>0) && bounceCount>0){
        if (reflectionGlossiness > 0) N = GlossyNormal(hInfo.N,reflectionGlossiness);
        else N = hInfo.N;
        // initialize hitinfo
        HitInfo hitinfo;
        hitinfo.Init();
        Vec3f reflected_dir = (N*2.0*V.Dot(N)-V).GetNormalized();
        Ray reflected_ray(P, reflected_dir);
        Color reflected_color(0.0);
        if(TraceRay(reflected_ray, hitinfo)){
            reflected_color = hitinfo.node->GetMaterial()->Shade(reflected_ray,hitinfo,lights,bounceCount-1);
        }
        else{
            reflected_color = environment.SampleEnvironment(reflected_ray.dir);
        }
        output += (mtl->reflection.GetColor() + R*mtl->refraction.GetColor())*reflected_color;
    }

    // indirect illumination
    if(bounceCount>0){
        N = hInfo.N;
        Color diffuse_color(0.0);
        float phi, sin1, cos1, sin2, cos2;
        HitInfo hitinfo;
        Vec3f X, Y, Z, S;
        Matrix3f CA;
        // diffuse reflection
        // compute coordinate axes
        Z = N.GetNormalized();
        S = Vec3f(0,0,1);
        if(Z.z>0.8) S = Vec3f(0,1,0);
        X = S ^ Z;
        Y = X ^ Z;
        X.Normalize();
        Y.Normalize();
        CA = Matrix3f(X, Y, Z);
        // sample ray
        cos1 = sqrtf(1-((float)rand())/RAND_MAX);
        sin1 = sqrtf(1.0 - cos1*cos1);
        phi = 2*M_PI*((float)rand())/RAND_MAX;
        sin2 = sin(phi);
        cos2 = cos(phi);
        Vec3f diffuse_dir(sin1*cos2, sin1*sin2, cos1);
        diffuse_dir = CA*diffuse_dir;
        Ray diffuse_ray(P, diffuse_dir);
        hitinfo.Init();
        if(TraceRay(diffuse_ray, hitinfo)){
            diffuse_color += hitinfo.node->GetMaterial()->Shade(diffuse_ray,hitinfo,lights,bounceCount-1);
        }
        else{
            diffuse_color += environment.SampleEnvironment(diffuse_ray.dir);
        }
        diffuse = N.Dot(diffuse_dir);
        if(diffuse>0){
            output += (diffuse_color * Kd);
        }
        // specular reflection
        if(Ks.Sum()>0){
            Color specular_color(0.0);
            // float alphabp = alpha;
            float alphabp = alpha/2;
            // compute coordinate axes
            Z = (N*2.0*V.Dot(N)-V).GetNormalized();
            S = Vec3f(0,0,1);
            if(Z.z>0.8) S = Vec3f(0,1,0);
            X = S ^ Z;
            Y = X ^ Z;
            X.Normalize();
            Y.Normalize();
            CA = Matrix3f(X, Y, Z);
            // sample ray
            cos1 = pow((float)rand()/RAND_MAX,1.0/(alphabp+1.0));
            sin1 = sqrtf(1.0 - cos1*cos1);
            phi = 2*M_PI*((float)rand())/RAND_MAX;
            sin2 = sin(phi);
            cos2 = cos(phi);
            Vec3f specular_dir(sin1*cos2, sin1*sin2, cos1);
            specular_dir = CA*specular_dir;
            Ray specular_ray(P, specular_dir);
            hitinfo.Init();
            if(TraceRay(specular_ray, hitinfo)){
                specular_color += hitinfo.node->GetMaterial()->Shade(specular_ray,hitinfo,lights,bounceCount-1);
            }
            else{
                specular_color += environment.SampleEnvironment(specular_ray.dir);
            }
            // phong
            // RR = (N*2.0*specular_dir.Dot(N)-specular_dir).GetNormalized();
            // specular = V.Dot(RR);
            // blinn phong
            H = (specular_dir+V).GetNormalized();
            specular = N.Dot(H);
            if(specular>0){
                output += (specular_color * Ks * (alphabp+2.0) / (alphabp+1.0));
            }
        }    
    }

    return output;
}

void Render(pixelIterator &pID){
	// initialize image variables
    float L = camera.focaldist;
    float H = 2*L*tan(camera.fov/2*(M_PI/180));
    float W = H*(float)camera.imgWidth/camera.imgHeight;
    Vec3f topLeft = Vec3f(-W/2, H/2, -L);
    float pw = W/camera.imgWidth;
    float ph = -H/camera.imgHeight;
    // compute coordinate axes
    Vec3f Z = -1*(camera.dir);
    Vec3f X = camera.up ^ Z;
    Vec3f Y = camera.up;
    X.Normalize();
    Y.Normalize();
    Z.Normalize();
    Matrix3f CA(X, Y, Z);
    // declare camera, pixel variables
    Vec3f pixel, uvw, camerapos, dir;
    int x, y, n, num_sample;
    HitInfo hitinfo;
    bool hit;
    float s, r, theta, cx, cy, px, py, hitz, threshold = 1e-4f;    
    int min_sample = 1;
    int max_sample = 1;  
    // for every pixel
    while(pID.PixelID(x,y,n)){
    	// compute camera to pixel ray
        pixel = topLeft + Vec3f(x*pw, y*ph, 0);
        uvw = Vec3f((float)x/camera.imgWidth, (float)y/camera.imgHeight, 0);
        // initialize hitinfo
        hit = false;
        hitz = BIGFLOAT;
        num_sample = 1;
        Color pColor(0.0);
        Color mean(0.0), m2(0.0);
        while(num_sample <= max_sample){
            // generate camera position sample
            if(camera.dof>0){
                s = ((float)rand())/RAND_MAX;
                r = camera.dof*sqrt(s);
                theta = 2*M_PI*((float)rand())/RAND_MAX;
                cx = r*cos(theta);
                cy = r*sin(theta);
                camerapos = Vec3f(cx,cy,0);        
            }
            else camerapos = Vec3f(0);
            // generate the halton sample in pixel
            px = Halton(num_sample,2)*pw;
            py = Halton(num_sample,3)*ph;
            // generate camera to pixel ray
            dir = pixel + Vec3f(px,py,0)-camerapos;
            camerapos = CA*camerapos;
            dir = CA*dir;
            Ray ray_pixel(camera.pos+camerapos, dir);
            ray_pixel.dir.Normalize();
            // trace the ray
            hitinfo.Init();
            if(TraceRay(ray_pixel, hitinfo)){
                pColor = hitinfo.node->GetMaterial()->Shade(ray_pixel,hitinfo,lights,2);
                hit = true;
                hitz = min(hitz, hitinfo.z);
            }
            else pColor = background.Sample(uvw);

            num_sample++;
            Color delta = pColor - mean;
            mean += delta / (float)(num_sample-1);
            m2 += delta * (pColor - mean);
            if(hit && num_sample>min_sample && num_sample<max_sample){
                Color variance = m2 / (float)(num_sample - 2);
                if(variance.Gray() < threshold) break;
            }
        }

        if(hit) updatePixel(n, hitz, num_sample-1, mean);
        else updatePixel(n, hitinfo.z, num_sample-1, background.Sample(uvw));
        renderImage.IncrementNumRenderPixel(1);
    }
}

void BeginRender()
{
	pID.Init();
    unsigned int n = std::thread::hardware_concurrency();
	cout << "Rendering the scene using " << n << " concurrent threads.\n";
    vector<thread> threads;
    for(int j=0; j<n; j++){
        thread t(Render, ref(pID));
        threads.push_back(move(t));
    }
    for(int j=0; j<n; j++){
    	if(threads[j].joinable())
        	threads[j].join();
    }
    renderImage.ComputeZBufferImage();
    // renderImage.SaveZImage("prj12_output/zbuffer.png");
    renderImage.SaveImage("../test.png");
    // renderImage.ComputeSampleCountImage();
}   

void StopRender(){
   
}

int main(int argc, const char * argv[]) {
    // load the scene file
    // int V0 = LoadScene("scene.xml");
    int V0 = LoadScene("scene_new.xml");
    ShowViewport();
    return 0;
}