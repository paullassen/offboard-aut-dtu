#include <iostream>

template <class T>
class Triplet {
 private:
  T x;
  T y;
  T z;

 public:
  Triplet() : x(0), y(0), z(0) {}
  Triplet(T w_) : x(w_), y(w_), z(w_) {}
  Triplet(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}
  Triplet(const Triplet<float>& obj)
      : x(obj.get_x()), y(obj.get_y()), z(obj.get_z()) {}
  Triplet(const Triplet<double>& obj)
      : x(obj.get_x()), y(obj.get_y()), z(obj.get_z()) {}
  Triplet(const Triplet<int>& obj)
      : x(obj.get_x()), y(obj.get_y()), z(obj.get_z()) {}

  void set_x(T x_) { x = x_; }
  void set_y(T y_) { y = y_; }
  void set_z(T z_) { z = z_; }

  void set(T x_, T y_, T z_) {
    set_x(x_);
    set_y(y_);
    set_z(z_);
  }

  void set(const Triplet<T>& obj) {
    set_x(obj.get_x());
    set_y(obj.get_y());
    set_z(obj.get_z());
  }

  T get_x(void) const { return x; }
  T get_y(void) const { return y; }
  T get_z(void) const { return z; }

  void print() {
    std::cout << "\n-------------" << std::endl;
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "z: " << z << std::endl;
    std::cout << "-------------" << std::endl;
  }

  Triplet& operator+=(const Triplet& obj) {
    x += (T)obj.x;
    y += (T)obj.y;
    z += (T)obj.z;
    return *this;
  }

  Triplet operator+(const Triplet& obj) {
    Triplet<T> result(*this);
    result += obj;
    return result;
  }

  Triplet& operator-=(const Triplet& obj) {
    x -= (T)obj.x;
    y -= (T)obj.y;
    z -= (T)obj.z;
    return *this;
  }

  Triplet operator-(const Triplet& obj) {
    Triplet<T> result(*this);
    result -= obj;
    return result;
  }

  Triplet& operator*=(const Triplet& obj) {
    x *= (T)obj.x;
    y *= (T)obj.y;
    z *= (T)obj.z;
    return *this;
  }

  Triplet operator*(const Triplet& obj) {
    Triplet<T> result(*this);
    result *= obj;
    return result;
  }

  Triplet operator/=(const Triplet& obj) {
    x /= (T)obj.x;
    y /= (T)obj.y;
    z /= (T)obj.z;
    return *this;
  }

  Triplet operator/(double obj) {
    Triplet<T> div(obj);
    Triplet<T> result(*this);
    result /= div;
    return result;
  }
};

int main() {
  Triplet<float> zero;
  Triplet<double> rnd(1, 2, 3);
  Triplet<int> inty(2, 5, 11.9);
  zero.print();
  rnd.print();
  zero.set_y(3);
  zero.set_x(7);
  Triplet<float> noob(zero);
  zero.print();
  noob.print();
  noob += rnd;
  noob.print();
  Triplet<float> sum(noob + rnd);
  sum.print();
  inty.print();
  Triplet<float> div(inty / 2);
  div.print();
}
/*

  Triplet operator+(const Triplet& obj) {
    Triple<T> result(x + (T)obj.get_x(), y + (T)obj.get_y(), z + (T)z.get(y));
    return result;
  }

  Triplet operator-(const Triplet& obj) {
    Triple<T> result(x - (T)obj.get_x(), y - (T)obj.get_y(), z - (T)z.get(y));
    return result;
  }

  Triplet operator*(const Triplet& obj) {
    Triple<T> result(x * (T)obj.get_x(), y * (T)obj.get_y(), z * (T)z.get(y));
    return result;
  }

  Triplet operator/(double obj) {
    Triple<T> result(x / (T)obj, y / (T)obj, z / (T)obj);
    return result;
  }

  Triplet operator/(float obj) {
    Triple<T> result(x / (T)obj, y / (T)obj, z / (T)obj);
    return result;
  }

  Triplet operator/(int obj) {
    Triple<T> result(x / (T)obj, y / (T)obj, z / (T)obj);
    return result;
  }
};
*/
