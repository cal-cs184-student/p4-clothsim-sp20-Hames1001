#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
    /*
  
     */
    // L_ambient
    vec3 L_ambient = vec3(0.03, 0.03, 0.03);
    
    //L_diffuse
    vec3 n = vec3(v_normal.xyz);
    vec3 pos = vec3(v_position.xyz);
    vec3 len = u_light_pos - pos;
    float r = sqrt(pow(len.x, 2) + pow(len.y, 2) + pow(len.z, 2));
    float maxVal = max(0, dot(n, len / r));
    float radiusSq = r * r;
    float diffuseCoefficient = 1.0;
    vec3 L_diffuse = diffuseCoefficient * (u_light_intensity / radiusSq) * maxVal;
    
    //L_specular
    float k_s = 0.4;
    
    vec3 viewpoint = u_cam_pos - pos;
    vec3 bisector = viewpoint + len;
    vec3 h = bisector / sqrt(pow(bisector.x, 2) + pow(bisector.y, 2) + pow(bisector.z, 2));
    float specularMax = pow(max(0, dot(n, h)), 50.0);
    
    
    vec3 L_specular = k_s * (u_light_intensity / radiusSq) * specularMax;
    
    
   // out_color =
    vec3 L = L_ambient + L_diffuse + L_specular;
    out_color = vec4(L.xyz, 1);
  out_color.a = 1;
}

