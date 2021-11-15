#version 450
layout(location = 0) in vec3 v_Position;
layout(location = 0) out vec4 o_Target;

layout(set = 2, binding = 0) uniform WireframeMaterial_color {
    vec4 color;
};
void main() {
    # ifdef WIREFRAMEMATERIAL_DASHED
        float c = 1.0;
        float pos_x = sin(v_Position.x*14.0)*14.0;
        float pos_y = sin(v_Position.y*14.0)*14.0;
        if (pos_x != 1.0) {
          c *= pos_x;
        }
        if (pos_y != 1.0) {
          c *= pos_y;
        }
        o_Target = vec4(color.rgb, c);
    # else
        o_Target = vec4(color.x, color.y, color.z, 1.0);
    # endif
}
