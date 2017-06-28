attribute vec4 v_coord;
attribute vec2 v_texcoord;
varying vec2 f_texcoord;
uniform mat4 mvp;

void main(void) {
  gl_Position = mvp * v_coord;
  f_texcoord = vec2(v_texcoord.x, 1-v_texcoord.y);
}
