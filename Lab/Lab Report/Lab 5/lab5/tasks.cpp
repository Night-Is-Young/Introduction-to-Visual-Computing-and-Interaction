#include "Labs/5-Visualization/tasks.h"

#include <numbers>
#include <iostream>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {

    glm::vec4 black(0, 0, 0, 1), backwhite(1, 1, 1, 0), frontwhite(1, 1, 1, 1), blue(0.4, 0.4, 0.8, 1), red(0.8, 0.4, 0.4, 1), backred(0.5, 0.1, 0.1, 0.4), lightgray(0.8, 0.8, 0.8, 0.5), darkgray(0.4, 0.4, 0.4, 1);
    struct CoordinateStates {
        // your code here
        std::vector<std::string>        labelName = { "cylinders", "displacement", "horsepower", "weight", "acceleration", "mileage", "year" };
        std::vector<int>                labelIdx  = { 0, 1, 2, 3, 4, 5, 6 };
        int                             mainIdx   = 0;
        int                             dataNum   = 7, dataSize;
        std::vector<std::vector<float>> dataTable; 
        std::vector<float>              minData, maxData;

        std::vector<std::vector<std::pair<float, float>>> dataPlotPos;
        const float minY = 0.15f, maxY = 0.85f;
        const float yShift = 0.05f, rectWidth = 0.015f, axisWidth = 2.0f, frameWidth = 1.5f;
        const float textShift = 0.025f, lineHeight = 0.01f;

        const float clickShift = 0.06f;
        bool        labelSwap  = false;
        glm::vec2   startPos, endPos;
        int         startIdx = -1, endIdx = -1;

        CoordinateStates(std::vector<Car> const& data) {
            dataSize = data.size();
            for (auto car : data) {
                std::vector<float> carData;
                carData.push_back(car.cylinders);
                carData.push_back(car.displacement);
                carData.push_back(car.horsepower);
                carData.push_back(car.weight);
                carData.push_back(car.acceleration);
                carData.push_back(car.mileage);
                carData.push_back(car.year);
                dataTable.push_back(carData);
            }
            minData = dataTable[0];
            maxData = dataTable[0];
            for (size_t i = 0; i < dataSize; ++i) {
                for (size_t j = 0; j < dataNum; ++j) {
                    minData[j] = std::min(dataTable[i][j], minData[j]);
                    maxData[j] = std::max(dataTable[i][j], maxData[j]);
                }
            }
        }

        void ConstructPlotPos() {
            dataPlotPos.clear();
            for (size_t i = 0; i < dataSize; ++i) {
                std::vector<std::pair<float, float>> line;
                for (int j = 0; j < dataNum; ++j) {
                    int   k    = labelIdx[j];
                    float yPos = minY + (maxData[k] - dataTable[i][k]) / (maxData[k] - minData[k]) * (maxY - minY);
                    line.push_back(std::pair<float, float>((2 * j + 1) / (2.0f * dataNum), yPos));
                }
                dataPlotPos.push_back(line);
            }
        }

        void Paint(Common::ImageRGB & input) {
            ConstructPlotPos();
            int idx = mainIdx;
            std::sort(dataPlotPos.begin(), dataPlotPos.end(), [idx](const std::vector<std::pair<float, float>> & a, const std::vector<std::pair<float, float>> & b) {
                return a[idx].second < b[idx].second;
            });

            SetBackGround(input, backwhite);
            for (size_t i = 0; i < dataSize; ++i) {
                float     s        = 1.0f * i / dataSize;
                float     t        = 0.5 + 4 * std::pow(s - 0.5, 3);
                glm::vec4 curColor = t * blue + (1 - t) * red;
                glm::vec2 from(dataPlotPos[i][0].first, dataPlotPos[i][0].second), to;
                for (size_t j = 1; j < dataNum; ++j) {
                    to = glm::vec2(dataPlotPos[i][j].first, dataPlotPos[i][j].second);
                    DrawLine(input, curColor, from, to, 0.1f);
                    from = to;
                }
            }
            for (size_t i = 0; i < dataNum; ++i) {
                float     centerX = (i * 2 + 1) * 1.0f / (dataNum * 2);
                glm::vec2 leftTop(centerX - rectWidth / 2, minY - yShift), size(rectWidth, maxY - minY + 2 * yShift);
                DrawLine(input, darkgray, glm::vec2(centerX, minY - yShift), glm::vec2(centerX, maxY + yShift), 2.0f);
                DrawFilledRect(input, (i == idx) ? backred : lightgray, leftTop, size);
                DrawRect(input, frontwhite, leftTop, size, frameWidth);
            }
            for (size_t i = 0; i < dataNum; ++i) {
                float centerX = (i * 2 + 1) * 1.0f / (dataNum * 2);
                PrintText(input, black, glm::vec2(centerX, minY - yShift - textShift * 2), lineHeight, labelName[labelIdx[i]]);
                PrintText(input, black, glm::vec2(centerX, minY - yShift - textShift), lineHeight, std::to_string((int) maxData[labelIdx[i]]));
                PrintText(input, black, glm::vec2(centerX, maxY + yShift + textShift), lineHeight, std::to_string((int) minData[labelIdx[i]]));
            }
        }

        void ResetLabelSwap() {
            startIdx  = -1;
            endIdx    = -1;
            labelSwap = false;
        }

        bool Update(InteractProxy const & proxy) {
            if (! proxy.IsHovering()) return false;
            if (proxy.IsClicking()) {
                glm::vec2 clickPos = proxy.MousePos();
                for (size_t i = 0; i < dataNum; ++i) {
                    float centerX = (i * 2 + 1) * 1.0f / (dataNum * 2);
                    if (std::abs(clickPos.x - centerX) < clickShift && clickPos.y > (minY - clickShift) && clickPos.y < (maxY + clickShift)) {
                        mainIdx      = i;
                        return true;
                    }
                }
            }
            if (proxy.IsClicking(false)) {
                if (!labelSwap) {
                    startPos = proxy.MousePos();
                    for (size_t i = 0; i < dataNum; ++i) {
                        float centerX = (i * 2 + 1) * 1.0f / (dataNum * 2);
                        if (std::abs(startPos.x - centerX) < clickShift && startPos.y > (minY - clickShift) && startPos.y < (maxY + clickShift)) {
                            startIdx = i;
                        }
                    }
                    if (startIdx != -1) labelSwap = true;
                    return false;
                } else {
                    endPos  = proxy.MousePos();
                    for (size_t i = 0; i < dataNum; ++i) {
                        float centerX = (i * 2 + 1) * 1.0f / (dataNum * 2);
                        if (std::abs(endPos.x - centerX) < clickShift && endPos.y > (minY - clickShift) && endPos.y < (maxY + clickShift)) {
                            endIdx = i;
                        }
                    }
                    if (endIdx != -1 && startIdx != endIdx) {
                        if (startIdx == mainIdx) mainIdx = endIdx;
                        if (endIdx == mainIdx) mainIdx = startIdx;
                        std::swap(labelIdx[startIdx], labelIdx[endIdx]);
                        ResetLabelSwap();
                        return true;
                    } else {
                        ResetLabelSwap();
                        return false;
                    }
                }
            }
        }
    };

    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        // your code here
        // for example: 
        //   static CoordinateStates states(data);
        //   SetBackGround(input, glm::vec4(1));
        //   ...
        static CoordinateStates states(data);                  // initialize
        bool                    change = states.Update(proxy); // update according to user input
        if (! force && ! change) return false;                 // determine to skip repainting
        states.Paint(input);                                   // visualize
        return true;
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        // your code here
        int width = output.GetSizeX(), height = output.GetSizeY();
        float     x, y, dt_x, dt_y, dt;
        float     forward_total, backward_total, weight;
        glm::vec3 forward_sum, backward_sum;
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < height; ++j) {
                x             = i;
                y             = j;
                forward_sum   = glm::vec3(0);
                forward_total = 0;
                for (int k = 0; k < step; ++k) {
                    glm::vec2 v = field.At((int) x, (int) y);
                    dt_x        = 0;
                    dt_y        = 0;
                    if (v.x > 0) {
                        dt_x = ((std::floor(x) + 1) - x) / v.x;
                    } else if (v.x < 0) {
                        dt_x = (x - (std::ceil(x) - 1)) / (-v.x);
                    }
                    if (v.y > 0) {
                        dt_y = ((std::floor(y) + 1) - y) / v.y;
                    } else if (v.y < 0) {
                        dt_y = (y - (std::ceil(y) - 1)) / (-v.y);
                    }
                    if (v.x == 0 && v.y == 0) dt = 0;
                    else dt = std::min(dt_x, dt_y);
                    x = std::min(std::max(x + v.x * dt, 0.0f), width - 1.0f);
                    y = std::min(std::max(y + v.y * dt, 0.0f), height - 1.0f);
                    weight = std::pow(std::cos(0.46 * k), 2);
                    forward_sum += noise.At((int) x, int(y)) * weight;
                    forward_total += weight;
                }
                x              = i;
                y              = j;
                backward_sum   = glm::vec3(0);
                backward_total = 0;
                for (int k = 1; k < step; ++k) {
                    glm::vec2 v = (-1.0f) * field.At((int) x, (int) y);
                    dt_x        = 0;
                    dt_y        = 0;
                    if (v.x > 0) {
                        dt_x = ((std::floor(x) + 1) - x) / v.x;
                    } else if (v.x < 0) {
                        dt_x = (x - (std::ceil(x) - 1)) / (-v.x);
                    }
                    if (v.y > 0) {
                        dt_y = ((std::floor(y) + 1) - y) / v.y;
                    } else if (v.y < 0) {
                        dt_y = (y - (std::ceil(y) - 1)) / (-v.y);
                    }
                    if (v.x == 0 && v.y == 0) dt = 0;
                    else dt = std::min(dt_x, dt_y);
                    x      = std::min(std::max(x + v.x * dt, 0.0f), width - 1.0f);
                    y      = std::min(std::max(y + v.y * dt, 0.0f), height - 1.0f);
                    weight = std::pow(std::cos(-0.46 * k), 2);
                    backward_sum += noise.At((int) x, int(y)) * weight;
                    backward_total += weight;
                }
                output.At(i, j) = (forward_sum + backward_sum) / (forward_total + backward_total);
            }
        }
    }
}; // namespace VCX::Labs::Visualization