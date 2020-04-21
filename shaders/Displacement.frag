#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv).r;
}

void main() {
  // YOUR CODE HERE
    float k_h = u_height_scaling;
    float k_n = u_normal_scaling;
    vec2 uwv = vec2(v_uv.x + (1 / u_texture_2_size.x), v_uv.y);
    vec2 uvh = vec2(v_uv.x, v_uv.y + (1 / u_texture_2_size.y));
       
    float H_uv = h(v_uv);
    float H_uwv = h(uwv);
    float H_uvh = h(uvh);
       
    float dU = 7 * (H_uwv - H_uv) * k_h * k_n;
    float dV = 7 * (H_uvh - H_uv) * k_h * k_n;
       
    vec3 n_o = vec3(-1 * dU, -1 * dV, 1);
       
    vec3 b = cross(v_normal.xyz, v_tangent.xyz);
    mat3 TBN = mat3(v_tangent.xyz, b.xyz, v_normal.xyz);
       
    vec3 n_d = TBN * n_o;
    
    // L_ambient
    vec3 L_ambient = vec3(0.03, 0.03, 0.03);
       
    //L_diffuse
    vec3 pos = vec3(v_position.xyz);
    vec3 len = u_light_pos - pos;
    float r = sqrt(pow(len.x, 2) + pow(len.y, 2) + pow(len.z, 2));
    float maxVal = max(0, dot(n_d, len / r));
    float radiusSq = r * r;
    float diffuseCoefficient = 1.0;
    vec3 L_diffuse = diffuseCoefficient * (u_light_intensity / radiusSq) * maxVal;
       
    //L_specular
    float k_s = 0.4;
    vec3 viewpoint = u_cam_pos - pos;
    vec3 bisector = viewpoint + len;
    vec3 h = bisector / sqrt(pow(bisector.x, 2) + pow(bisector.y, 2) + pow(bisector.z, 2));
    float specularMax = pow(max(0, dot(n_d, h)), 50.0);
       
       
    vec3 L_specular = k_s * (u_light_intensity / radiusSq) * specularMax;
       
    vec3 L = L_ambient + L_diffuse + L_specular;
  out_color = vec4(L.xyz, 1);
  out_color.a = 1;
}

