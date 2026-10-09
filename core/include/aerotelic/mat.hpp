#pragma once

#include <vector>
#include <stdexcept>
#include <cmath>

namespace aerotelic {

// Minimal dense, row-major matrix. Only the operations core/nurbs.cpp
// actually needs -- not a general linear-algebra type.
class Mat {
public:
	Mat() : rows_(0), cols_(0) {}
	Mat(int rows, int cols, double fill = 0.0) : rows_(rows), cols_(cols), data_(static_cast<size_t>(rows) * cols, fill) {}

	static Mat FromRows(const std::vector<std::vector<double>>& rows) {
		int n = static_cast<int>(rows.size());
		int d = n > 0 ? static_cast<int>(rows[0].size()) : 0;
		Mat m(n, d);
		for (int i = 0; i < n; ++i) {
			if (static_cast<int>(rows[i].size()) != d) throw std::invalid_argument("ragged rows");
			for (int j = 0; j < d; ++j) m(i, j) = rows[i][j];
		}
		return m;
	}

	int rows() const { return rows_; }
	int cols() const { return cols_; }

	double& operator()(int i, int j) { return data_[static_cast<size_t>(i) * cols_ + j]; }
	double operator()(int i, int j) const { return data_[static_cast<size_t>(i) * cols_ + j]; }

	std::vector<double> row(int i) const {
		std::vector<double> r(cols_);
		for (int j = 0; j < cols_; ++j) r[j] = (*this)(i, j);
		return r;
	}

	void set_row(int i, const std::vector<double>& r) {
		if (static_cast<int>(r.size()) != cols_) throw std::invalid_argument("row size mismatch");
		for (int j = 0; j < cols_; ++j) (*this)(i, j) = r[j];
	}

	// Appends row `r` at position `index` (0..rows_), shifting the rest down.
	Mat InsertRow(int index, const std::vector<double>& r) const {
		Mat out(rows_ + 1, cols_);
		for (int i = 0; i < index; ++i) out.set_row(i, row(i));
		out.set_row(index, r);
		for (int i = index; i < rows_; ++i) out.set_row(i + 1, row(i));
		return out;
	}

	Mat RemoveRow(int index) const {
		Mat out(rows_ - 1, cols_);
		for (int i = 0; i < index; ++i) out.set_row(i, row(i));
		for (int i = index + 1; i < rows_; ++i) out.set_row(i - 1, row(i));
		return out;
	}

	// Column stack: appends a single column built from `col`.
	Mat WithColumn(const std::vector<double>& col) const {
		if (static_cast<int>(col.size()) != rows_) throw std::invalid_argument("column size mismatch");
		Mat out(rows_, cols_ + 1);
		for (int i = 0; i < rows_; ++i) {
			for (int j = 0; j < cols_; ++j) out(i, j) = (*this)(i, j);
			out(i, cols_) = col[i];
		}
		return out;
	}

private:
	int rows_;
	int cols_;
	std::vector<double> data_;
};

inline double VecNorm(const std::vector<double>& v) {
	double s = 0.0;
	for (double x : v) s += x * x;
	return std::sqrt(s);
}

inline std::vector<double> VecSub(const std::vector<double>& a, const std::vector<double>& b) {
	std::vector<double> out(a.size());
	for (size_t i = 0; i < a.size(); ++i) out[i] = a[i] - b[i];
	return out;
}

} // namespace aerotelic
