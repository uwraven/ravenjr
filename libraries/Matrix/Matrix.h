/*
    Matrix data structure and arithmetic
    7/26/19 Matt Vredevoogd
*/

#ifndef Matrix_H
#define Matrix_H

#include <Arduino.h>
#include <ArduinoSTL.h>
#include <vector>

class Matrix {
    private:
        int m_rows;
        int m_cols;
        static constexpr double eps = 10e-3;
        vector<vector<double>> m_array;

        bool validIndex(unsigned _row, unsigned _col) {
            return (_row < m_rows && _row >= 0 && _col >= 0 && _col < m_cols);
        }

        void fill(double val) {
            // assumes the constructor has been called and
            // both matrix dimensions have been set
            for (unsigned row = 0; row < m_rows; row++) {
                for (unsigned col = 0; col < m_cols; col++) {
                    m_array[row][col] = val;
                }
            }
        }

    public:
        // Constructors

        Matrix(unsigned _rows, unsigned _cols) {
            // TODO
            // This is a potentially dangerous implementation of a matrix class
            // because it utilizes dynamically allocated vector containers. 
            // The required memory is not known at compile time, and the Arduino
            // is therefore susceptible to running out of memory during flight.

            m_rows = _rows;
            m_cols = _cols;

            // resize row vector
            m_array.resize(_rows);

            // iterate through rows
            for (unsigned row = 0; row < m_rows; row++) {
                // resize column vectors
                m_array[row].resize(_cols);
                for (unsigned col = 0; col < m_cols; col++) {
                    // init array
                    m_array[row][col] = 0.0;
                }
            }
        }

        Matrix(unsigned _rows, unsigned _cols, double val) {
            m_rows = _rows;
            m_cols = _cols;
            m_array.resize(_rows);
            for (unsigned row = 0; row < m_rows; row++) {
                m_array[row].resize(_cols);
                for (unsigned col = 0; col < m_cols; col++) {
                    m_array[row][col] = val;
                }
            }
        }

        Matrix(vector<vector<double>> m_arr) {
            m_rows = m_arr.size();
            m_cols = m_arr[0].size();
            m_array.resize(m_rows);
            for (unsigned row = 0; row < m_rows; row++) {
                m_array[row].resize(m_cols);
                for (unsigned col = 0; col < m_cols; col++) {
                    m_array[row][col] = m_arr[row][col];
                }
            }
        }

        Matrix diag() {
            Matrix diag{1, m_rows};
            if (m_rows == m_cols) {
                // is a square matrix
                for (unsigned i = 0; i < m_rows; i++) {
                    diag[1][i] = m_array[i][i];
                }
            }
            return diag;
        }

        // Static identity matrix
        static Matrix identity(unsigned _rows, unsigned _cols) {
            Matrix eye{_rows, _cols};
            for (unsigned i = 0; i < _rows; i++) {
                eye[i][i] = 1.0;
            }
            return eye;
        }

        // // Destructor
        // ~Matrix();

        void print() {
            for (unsigned i = 0; i < m_rows; i++) {
                for (unsigned j = 0; j < m_cols; j++) {
                    Serial.print(m_array[i][j]);
                    Serial.print(F("  "));
                }
                Serial.println();
            }
        }

        // Methods
        void setAt(unsigned _row, unsigned _col, double val) {
            if (validIndex(_row, _col)) {
                m_array[_row][_col] = val;
            };
        }

        // void unsafeSetAt(unsigned _row, unsigned _col, double val) {
        //     m_array[m_cols * _row + _col] = val;
        // }

        // void rowAssign(double vars[_cols], unsigned row) {
        //     for (unsigned col = 0; col < _cols; col++) {
        //         this->setAt(row, col, vars[col]);
        //     }
        // }

        // void colAssign(double vars[_rows], unsigned col) {
        //     for (unsigned row = 0; row < _rows; row++) {
        //         this->setAt(col, row, vars[row]);
        //     }
        // }
        
        // Overloads
        Matrix Matrix::operator + (Matrix &B) {
            Serial.println(F("Adding"));
            Matrix result(m_rows, m_cols);
            if (m_rows != B.m_rows || m_cols != B.m_cols) {
                Serial.println(F("ERROR in MATRIX [+] OVERLOAD"));
                Serial.println(F("MATRICES ARE INCONGRUENT"));
            } else {
                for (unsigned i = 0; i < m_rows; i++) {
                    for (unsigned j = 0; j < m_cols; j++) {
                        double sum = m_array[i][j] + B[i][j];
                        result.setAt(i, j, sum);
                    }
                }
            }
            return result;
        }

        Matrix operator - (Matrix &B) {
            Matrix result(m_rows, m_cols);
            for (unsigned i = 0; i < m_rows; i++) {
                for (unsigned j = 0; j < m_cols; j++) {
                    double delt = m_array[i][j] - B[i][j];
                    result.setAt(i, j, delt);
                }
            }
            return result;
        }

        Matrix operator * (Matrix &B) {
            unsigned const r_cols = B.cols();
            Matrix result(m_rows, r_cols);
            if (cols() == B.rows()) {
                double sum = 0;
                for (unsigned row = 0; row < m_rows; row++) {
                    for (unsigned col = 0; col < r_cols; col ++) {
                        sum = 0;
                        for (unsigned index = 0; index < m_cols; index++) {
                            sum += m_array[row][index] * B[index][col];
                        }
                        result[row][col] = sum;
                    }
                }
            }
            return result;
        }

        Matrix operator * (double scalar) {
            Matrix result(m_rows, m_cols);
            for (unsigned row = 0; row < m_rows; row++) {
                for (unsigned col = 0; col < m_cols; col++) {
                    double prod = m_array[row][col] * scalar;
                    result[row][col] = prod;
                }
            }
            return result;
        }

        Matrix operator / (double scalar) {
            Matrix result(m_rows, m_cols);
            for (unsigned row = 0; row < m_rows; row++) {
                for (unsigned col = 0; col < m_cols; col++) {
                    double prod = m_array[row][col] / scalar;
                    result[row][col] = prod;
                }
            }
            return result;
        }

        // Matrix Matrix::operator = (const Matrix &m) {
        //     Serial.println("==");
        // }

        Matrix inv() {
            // Don't mutate original array
            Matrix mat(m_array);
            Matrix aug = Matrix::identity(m_rows, m_cols);

            // ii iterator represents diagonal index 
            for (unsigned ii = 0; ii < m_rows; ii++) {

                double divisor = mat[ii][ii];

                // Divide row and augmented row by divisor
                for (unsigned col = 0; col < m_cols; col++) {
                    double res = mat[ii][col] / divisor;
                    double augres = aug[ii][col] / divisor;
                    mat[ii][col] = res;
                    aug[ii][col] = augres;
                    // mat.setAt(ii, col, res);
                    // aug.setAt(ii, col, augres);
                }

                // subtract remaining rows, except at the pivot row ii
                for (unsigned row = 0; row < m_rows; row++) {
                    // only perform flops if non-zero
                    if (abs(mat(row, ii)) > eps & row != ii) {
                        double coeff = mat(row, ii);
                        // Iterate over columns and subtract the selected row times the selected element
                        for (unsigned col = 0; col < m_cols; col++) {
                            mat[row][col] = mat[row][col] - mat[ii][col] * coeff;
                            aug[row][col] = aug[row][col] - aug[ii][col] * coeff;
                            // mat.setAt(row, col, mat.get(row, col) - mat.get(ii, col) * coeff);
                            // aug.setAt(row, col, aug.get(row, col) - aug.get(ii, col) * coeff);
                        }
                    }
                }
            }
            return aug;
        }
        

        double operator () (const unsigned &row, const unsigned &col) {
            return m_array[row][col];
        };

        void operator = (const double &val) {
            fill(val);
        }

        vector<double> operator [] (const unsigned &index) {
            // Returns a read only value
            return m_array[index];
        }
        
        // Getters
        unsigned rows() {
            return m_rows;
        }
        unsigned cols() { 
            return m_cols;
        }


};

#endif