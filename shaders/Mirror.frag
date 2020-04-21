#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
    vec3 vPos = vec3(v_position.xyz);
    vec3 w_o = u_cam_pos - vPos;
    vec3 w_i = -1 * reflect(w_o.xyz, vec3(v_normal.xyz));
    out_color = texture(u_texture_cubemap, w_i);
  out_color.a = 1;
}
