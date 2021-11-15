#version 450

layout(location = 0) out vec4 o_Target;
layout(location = 0) in vec4 v_Color;

layout(set = 2, binding = 0) uniform PositionWireframeMaterial_x {
    vec4 color;
};
void main() {
    o_Target = v_Color;
}
