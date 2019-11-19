#include <set>
#include <cmath>
#include <string>
#include <random>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

// GNU C++11 4.9
// g++ -O2 -fno-stack-limit -x c++ detect_road_plane.cpp -o detect_road_plane

std::vector<float> cross_product(std::vector<std::vector<float>>& points) {
    float summ[3] = {0.0, 0.0, 0.0}, centroid[3];
    const int N = points.size();

    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < 3; ++c) {
            summ[c] += points[r][c];
        }
    }

    float k = 1.0 / N;
    for (int c = 0; c < 3; ++c) {
        centroid[c] = summ[c] * k;
    }

    float xx = 0.0, xy = 0.0, xz = 0.0, yy = 0.0, yz = 0.0, zz = 0.0;
    std::vector<float> r;
    for (const auto& point : points) {
         r = {point[0] - centroid[0], 
              point[1] - centroid[1], 
              point[2] - centroid[2]};
         xx += r[0] * r[0];
         xy += r[0] * r[1];
         xz += r[0] * r[2];
         yy += r[1] * r[1];
         yz += r[1] * r[2];
         zz += r[2] * r[2];
    }

    float det_x = yy * zz - yz * yz;
    float det_y = xx * zz - xz * xz;
    float det_z = xx * yy - xy * xy;

    float det_max = std::max({det_x, det_y, det_z});

    std::vector<float> dirr;
    if (det_max == det_x) {
        dirr = {det_x, xz*yz - xy*zz, xy*yz - xz*yy};
    } else if (det_max == det_y) {
        dirr = {xz*yz - xy*zz, det_y, xy*xz - yz*xx};
    } else {
        dirr = {xy*yz - xz*yy, xy*xz - yz*xx, det_z};
    }

    if (dirr[0]+dirr[1]+dirr[2] == 0.0) {
    	return {};
    }

    // make a unit normal vector
    float dirr_norm = 0.0;
    for (const auto& value : dirr) {
        dirr_norm += value * value;
    }
    dirr_norm = std::sqrt(dirr_norm);

    int c = 0;
    float d = 0.0;
    for (auto& value : dirr) {
        value = value / dirr_norm;
        d += -1.0 * value * points[0][c];
        c++;
    }
    dirr.push_back(d);

    return dirr;
}

std::vector<float> get_dists(std::vector<std::vector<float>>& points, std::vector<float>& v_n) {
    int N = points.size();
    std::vector<float> dists(N);

    for (int r = 0; r < N; ++r) {
        dists[r] = 0.0;
        for (int c = 0; c < 3; ++c) {
            dists[r] += points[r][c] * v_n[c];
        }
        dists[r] += v_n[3];
        dists[r] = std::abs(dists[r]);
        dists[r] /= std::sqrt(v_n[0]*v_n[0] + v_n[1]*v_n[1] + v_n[2]*v_n[2]);
    }

    return dists;
}

std::vector<float> ransac_regression(std::vector<std::vector<float>>& points, 
                                     const float& p, const int& max_iters) {
    const int N = points.size();

    int maxx = -1;
    std::vector<std::vector<float>> best_set;
    std::vector<float> v_n(4), dists(N);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, (N-1));

    for (int i = 0; i < std::min(max_iters, N / 2); ++i) {
        std::vector<std::vector<float>> chosen, closest;
        std::set<int> uniques;
        int n_closest;

        while (uniques.size() < 3) {
        	int idx = dis(gen);
        	chosen.push_back(points[idx]);
        	uniques.insert(idx);
        }

        v_n = cross_product(chosen);
        if (v_n.size() == 0) {
            continue;
        }
        
        dists = get_dists(points, v_n);
        for (int j = 0; j < N; ++j) {
            if (dists[j] <= p) {
                closest.push_back(points[j]);
            }
        }

        n_closest = closest.size();
        if (maxx < n_closest) {
            maxx = n_closest;
            best_set = closest;
            // if you want to make it dirty ;)
            // if (n_closest >= (N / 2)) {
            //     break;
            // }
        }

    }

    if (best_set.size() > 2) {
    	return cross_product(best_set);
    } else {
    	return {};
    }
}

int main() {

    /*
    ransac regression
    average computing time (~20k examples; 100 tries):
     - 6k iters ~3.7s: 
            [A]:-0.014723 (0.000250) [B]:-0.001545 (0.000224) [C]:0.999890 (0.000003) [D]:0.097575 (0.000426) 
     - 5k iters ~3.1s:
            [A]:-0.014702 (0.000238) [B]:-0.001554 (0.000221) [C]:0.999891 (0.000003) [D]:0.097542 (0.000441) 
     - 4k iters ~2.4s: 
            [A]:-0.014698 (0.000244) [B]:-0.001569 (0.000240) [C]:0.999891 (0.000003) [D]:0.097587 (0.000573) <<<
     - 3k iters ~1.8s:
            [A]:-0.014772 (0.000274) [B]:-0.001518 (0.000268) [C]:0.999890 (0.000004) [D]:0.097630 (0.000485) <<<
     - 2k iters ~1.3s:
            [A]:-0.014782 (0.000331) [B]:-0.001510 (0.000317) [C]:0.999890 (0.000004) [D]:0.097630 (0.000588)
     - 1k iters ~0.6s: 
            [A]:-0.014845 (0.000421) [B]:-0.001527 (0.000415) [C]:0.999888 (0.000006) [D]:0.097889 (0.000829)
    */

    std::ifstream input;
    input.open("input.txt");

    if (!input.is_open()) {
        std::exit(EXIT_FAILURE);
    }

    std::string str;
    int n = 0, max_iters = 3000;

    std::getline(input, str);
    const float p = std::stof(str);
    std::getline(input, str);
    const int N = std::stoi(str);
    std::vector<std::vector<float>> points(N);

    while (std::getline(input, str)) {
        std::istringstream iss(str);
        std::string token;
        while (std::getline(iss, token, '\t')) {
            points[n].push_back(std::atof(token.c_str()));
        }
        n++;
    }
    input.close();

    std::vector<float> out = ransac_regression(points, p, max_iters);
    if (out.size() > 0) {
    	std::ofstream output("output.txt");
	    for (auto &el : out) {

            // rounding
            el = std::round(el * 1e6) / 1e6;
            float abs_el = std::abs(el);
            if (abs_el == 0.0) el = abs_el;

	        output << std::to_string(el) << " ";
	    }
	    output.close();
	}

    return 0;
}
