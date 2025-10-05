#include <random>
#include <cmath>
#include <spdlog/spdlog.h>
#include <iostream>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;

namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        std::random_device rd;
        std::mt19937 gen((unsigned int)rd());
        std::uniform_real_distribution<float> dis(-0.5,0.5);
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                int graysacle = (color.r + dis(gen)) > 0.5 ? 1 : 0;
                output.At(x, y) = {
                    graysacle, graysacle, graysacle,
                };
            }
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        // your code here:
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color_input = input.At(x, y);
                glm::vec3 color_noise = noise.At(x, y);
                int       graysacle   = (color_input.r + color_noise.r) > 1 ? 1 : 0;
                output.At(x, y)       = {
                    graysacle,
                    graysacle,
                    graysacle,
                };
            }
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int mat[3][3] = {
            { 6, 8, 4 },
            { 1, 0, 3 },
            { 5, 2, 7 }
        };
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                float     color_judge = color.r * 9;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        int color_draw = color_judge >= mat[i][j] ? 1 : 0;
                        output.At(3 * x + i, 3 * y + j) = {
                            color_draw,
                            color_draw,
                            color_draw,
                        };
                    }
                }
            }
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        std::size_t                     row = input.GetSizeX(), col = input.GetSizeY();
        std::vector<std::vector<float>> colormat(row, std::vector<float>(col, 0));
        for (std::size_t x = 0; x < row; ++x) {
            for (std::size_t y = 0; y < col; ++y) {
                glm::vec3 color = input.At(x, y);
                colormat[x][y]  = color.r;
            }
        }
        for (std::size_t y = 0; y < col; ++y) {
            for (std::size_t x = 0; x < row; ++x) {
                int color       = colormat[x][y] > 0.5 ? 1 : 0;
                output.At(x, y) = {
                    color,
                    color,
                    color,
                };
                float delta = colormat[x][y] - (colormat[x][y] > 0.5 ? 1 : 0);
                if (x < row - 1) { colormat[x + 1][y] += delta * 7 / 16; }
                if (y < col - 1) {
                    colormat[x][y + 1] += delta * 5 / 16;
                    if (x > 0) { colormat[x - 1][y + 1] += delta * 3 / 16; }
                    if (x < row - 1) { colormat[x + 1][y + 1] += delta * 1 / 16; }
                }
            }
        }
    }

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        std::size_t row = input.GetSizeX(), col = input.GetSizeY();
        for (std::size_t x = 0; x < row; ++x)
            for (std::size_t y = 0; y < col; ++y) {
                int ave_counter = 0;
                glm::vec3 color(0.0f, 0.0f, 0.0f);
                for (int i = -1; i <= 1; i++) {
                    for (int j = -1; j <= 1; j++) {
                        if (x + i >= 0 && x + i < row && y + j >= 0 && y + j < col) {
                            ave_counter++;
                            color += input.At(x + i, y + j);
                        }
                    }
                }
                output.At(x, y) = {
                    color.r / ave_counter,
                    color.g / ave_counter,
                    color.b / ave_counter,
                };
            }
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        float G_x[3][3] = {
            { -1.0f, 0.0f, 1.0f },
            { -2.0f, 0.0f, 2.0f },
            { -1.0f, 0.0f, 1.0f }
        };
        float G_y[3][3] = {
            {  1.0f,  2.0f,  1.0f },
            {  0.0f,  0.0f,  0.0f },
            { -1.0f, -2.0f, -1.0f }
        };
        std::size_t row = input.GetSizeX(), col = input.GetSizeY();
        for (std::size_t x = 0; x < row; ++x)
            for (std::size_t y = 0; y < col; ++y) {
                glm::vec3 color_x(0.0f, 0.0f, 0.0f);
                glm::vec3 color_y(0.0f, 0.0f, 0.0f);
                for (int i = -1; i <= 1; i++) {
                    for (int j = -1; j <= 1; j++) {
                        if (x + i >= 0 && x + i < row && y + j >= 0 && y + j < col) {
                            glm::vec3 color_cur = input.At(x + i, y + j);
                            color_x += color_cur * G_x[i + 1][j + 1];
                            color_y += color_cur * G_y[i + 1][j + 1];
                        }
                    }
                }
                output.At(x, y) = {
                    sqrt(pow(color_x.r, 2) + pow(color_y.r, 2)),
                    sqrt(pow(color_x.g, 2) + pow(color_y.g, 2)),
                    sqrt(pow(color_x.b, 2) + pow(color_y.b, 2)),
                };
            }
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        std::size_t width  = inputFront.GetSizeX();
        std::size_t height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];
        memset(g, 0, sizeof(glm::vec3) * width * height);
        // set boundary condition
        for (std::size_t y = 0; y < height; ++y) {
            // set boundary for (0, y), your code: g[y * width] = ?
            g[y * width] = (glm::vec3) inputBack.At(offset.x, y + offset.y) - (glm::vec3) inputFront.At(0, y);
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
            g[y * width + width - 1] = (glm::vec3) inputBack.At(offset.x + width - 1, y + offset.y) - (glm::vec3) inputFront.At(width - 1, y);
        }
        for (std::size_t x = 0; x < width; ++x) {
            // set boundary for (x, 0), your code: g[x] = ?
            g[x] = (glm::vec3) inputBack.At(x + offset.x, offset.y) - (glm::vec3) inputFront.At(x, 0);
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
            g[(height - 1) * width + x] = (glm::vec3) inputBack.At(x + offset.x, offset.y + height - 1) - (glm::vec3) inputFront.At(x, height - 1);
        }

        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.At(x, y);
                output.At(x + offset.x, y + offset.y) = color;
            }
        delete[] g;
    }

    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        // your code here:
        int x0 = p0.x, y0 = p0.y, x1 = p1.x, y1 = p1.y;
        int x, y, dx, dy, d, F, cx, cy;
        bool f = std::abs(x1 - x0) < std::abs(y1 - y0);
        if (f) {
            std::swap(x0, y0);
            std::swap(x1, y1);
        }
        if (x0 > x1) {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }
        y  = y0;
        dx = 2 * (x1 - x0);
        dy = 2 * std::abs(y1 - y0);
        d  = dy - dx;
        F  = dy - dx / 2;
        for (x = x0; x <= x1; x++) {
            cx                = f ? y : x;
            cy                = f ? x : y;
            canvas.At(cx, cy) = {
                color.r,
                color.g,
                color.b,
            };
            if (F < 0) {
                F = F + dy;
            } else {
                y += (y1 > y0) ? 1 : -1;
                F = F + d;
            }
        }
    }

    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        // your code here:
        int        x0 = p0.x, y0 = p0.y, x1 = p1.x, y1 = p1.y, x2 = p2.x, y2 = p2.y;
        float      ymin, ymax;
        float      k1, k2;
        if (x2 < x1) {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        if (x1 < x0) {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }
        if (x0 != x1 && x2 != x0) {
            ymin = 1.0 * y0;
            ymax = 1.0 * y0;
            k1 = 1.0 * (y1 - y0) / (x1 - x0);
            k2 = 1.0 * (y2 - y0) / (x2 - x0);
            if (k1 > k2) { std::swap(k1, k2); }
            for (int x = x0; x <= std::min(x1, x2); x++) {
                for (int y = std::round(ymin); y < ymax+0.5; y++) {
                    canvas.At(x, y) = {
                        color.r,
                        color.g,
                        color.b,
                    };
                }
                ymin += k1;
                ymax += k2;
            }
        }
        if (x1 != x2 && x2 != x0) {
            ymin = 1.0 * y2;
            ymax = 1.0 * y2;
            k1 = 1.0 * (y1 - y2) / (x2 - x1);
            k2 = 1.0 * (y0 - y2) / (x2 - x0);
            if (k1 > k2) { std::swap(k1, k2); }
            for (int x = x2; x >= std::max(x1, x0); x--) {
                for (int y = std::round(ymin); y < ymax+0.5; y++) {
                    canvas.At(x, y) = {
                        color.r,
                        color.g,
                        color.b,
                    };
                }
                ymin += k1;
                ymax += k2;
            }
        }
    }

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        int output_row = output.GetSizeX(),
            output_col = output.GetSizeY(),
            input_row  = input.GetSizeX(),
            input_col  = input.GetSizeY();
        std::vector<std::vector<glm::vec3>> colormatrix(output_row * rate, std::vector<glm::vec3>(output_col * rate, { 0.0f, 0.0f, 0.0f }));
        for (int x = 0; x < output_row * rate; x++) {
            for (int y = 0; y < output_col * rate; y++) {
                float     sx = (x + 0.5f) * input_row / (output_row * rate);
                float     sy = (y + 0.5f) * input_col / (output_col * rate);
                int       x0 = std::clamp((int) std::floor(sx), 0, input_row - 1);
                int       y0 = std::clamp((int) std::floor(sy), 0, input_col - 1);
                int       x1 = std::clamp(x0 + 1, 0, input_row - 1);
                int       y1 = std::clamp(y0 + 1, 0, input_col - 1);
                float     dx = sx - x0, dy = sy - y0;
                glm::vec3 color_cur = (1 - dx) * (1 - dy) * (glm::vec3)(input.At(x0, y0)) 
                    + dx * (1 - dy) * (glm::vec3) input.At(x1, y0)
                    + dy * (1 - dx) * (glm::vec3) input.At(x0, y1)
                    + dx * dy * (glm::vec3) input.At(x1, y1);
                colormatrix[x][y] = color_cur;
            }
        }
        float inv = 1.0f / (rate * rate);
        for (int x = 0; x < output_row; x++) {
            for (int y = 0; y < output_col; y++) {
                glm::vec3 color { 0.0f, 0.0f, 0.0f };
                for (int i = 0; i < rate; i++) {
                    for (int j = 0; j < rate; j++) {
                        color += colormatrix[x * rate + i][y * rate + j];
                    }
                }
                color *= inv;
                output.At(x, y) = {
                    color.r,
                    color.g,
                    color.b,
                };
            }
        }
    }

    /******************* 7. Bezier Curve *****************/
    // Note: Please finish the function [DrawLine] before trying this part.
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        std::vector<glm::vec2> p = { points[0],
                                     points[1],
                                     points[2],
                                     points[3] };
        for (int i = 1; i <= 3; i++) {
            for (int j = 3; j >= i; j--) {
                p[j] = t * p[j] + (1 - t) * p[j - 1];
            }
        }
        return p[3];
    }
} // namespace VCX::Labs::Drawing2D