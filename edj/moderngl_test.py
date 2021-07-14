#!/usr/bin/env python3

import moderngl as gl
from ModernGL.ext import obj
from PIL import Image
from pyrr import Matrix44

class Renderer:
    def __init__(self, fmt = 'rgba', enable = gl.DEPTH_TEST):
        self.ctx = gl.create_standalone_context()
        self.ctx.enable(enable)

        self.fmts = {'rgb':(3,'f1','RGB'),'rgba':(4,'f1','RGBA'),'gs8':(1,'f1','L'),'gs16':(1,'u2','I;16')}
        self.fmt = self.fmts[fmt]

        self.program = None

    def compile_shaders(self, vertex_src, fragment_src):
        self.program = self.ctx.program(vertex_shader=vertex_src, fragment_shader=fragment_src)

    def create_framebuffer(self, dims):
        components = self.fmt[0]
        dtype = self.fmt[1]
        self.fb = self.ctx.texture(size=dims, components=components, dtype=dtype)
        self.fbo = self.ctx.framebuffer(color_attachments=[self.fb])
        self.fbo.use()

    def render(self):
        self.ctx.clear(1.0, 0.5, 0.0, 1.0, 1.0)

    def extract_framebuffer(self):
        components = self.fmt[0]
        dtype = self.fmt[1]
        img_fmt = self.fmt[2]

        data = self.fbo.read(components=components, attachment=0, dtype=dtype)

        image = Image.frombytes(img_fmt, self.fbo.size, data)
        return image

    def save_framebuffer(self, filename):
        image = self.extract_framebuffer()
        image.save(filename)

if __name__ == '__main__':
    r = Renderer()

    with open('shader_vert.glsl') as f:
        vert_shader = f.read()

    with open('shader_frag.glsl') as f:
        frag_shader = f.read()

    r.compile_shaders(vert_shader, frag_shader)
    r.create_framebuffer((960, 640))
    r.render()
    r.save_framebuffer('moderngl_test.png')
