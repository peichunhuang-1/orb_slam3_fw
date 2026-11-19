#pragma once
// Reader for the ORB-SLAM3 golden fixtures (tests/fixtures/orb_slam3/README.md): one value per line,
// "name rows cols v...", row-major; '#' lines are comments.
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#ifndef ORB_SLAM3_FIXTURE_DIR
#error "define ORB_SLAM3_FIXTURE_DIR (tests/fixtures/orb_slam3)"
#endif

namespace fixture {

inline std::string path(const std::string& relative) { return std::string(ORB_SLAM3_FIXTURE_DIR) + "/" + relative; }

class File {
public:
    explicit File(const std::string& relative) : name_(relative) {
        std::ifstream in(path(relative));
        if (!in) throw std::runtime_error("fixture not found: " + path(relative));
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream s(line);
            std::string key;
            int r = 0, c = 0;
            s >> key >> r >> c;
            Eigen::MatrixXd m(r, c);
            for (int i = 0; i < r; ++i)
                for (int j = 0; j < c; ++j)
                    if (!(s >> m(i, j))) throw std::runtime_error(relative + ": short value " + key);
            values_[key] = m;
        }
    }

    bool has(const std::string& key) const { return values_.count(key) != 0; }
    const Eigen::MatrixXd& operator[](const std::string& key) const {
        const auto it = values_.find(key);
        if (it == values_.end()) throw std::runtime_error(name_ + ": no value " + key);
        return it->second;
    }
    double scalar(const std::string& key) const { return (*this)[key](0, 0); }
    // Fixed-size float copy (the original's types)
    template <int R, int C> Eigen::Matrix<float, R, C> f(const std::string& key) const {
        const Eigen::MatrixXd& m = (*this)[key];
        if (m.rows() != R || m.cols() != C) throw std::runtime_error(name_ + ": wrong size " + key);
        return m.cast<float>();
    }
    const std::string& name() const { return name_; }

private:
    std::string name_;
    std::map<std::string, Eigen::MatrixXd> values_;
};

}  // namespace fixture
