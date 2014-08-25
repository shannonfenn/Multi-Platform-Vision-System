/**
 * @file Vector3.h
 * Contains template class Vector3 of type V
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 * REMOVED STREAMABLE
 */

#ifndef __Vector3_h__
#define __Vector3_h__


#include <math.h>
#include <iostream>

/** This class represents a 3-vector */
template <class V = float> class Vector3
{
public:

  /** The vector values */
  V x,y,z;

  /** Default constructor 4 gcc. */
  Vector3<V>():x(),y(),z()
  {
  }

  /** Default constructor. */
  Vector3<V>(V x, V y, V z):x(x),y(y),z(z)
  {
  }

  /** Assignment operator
  *\param other The other vector that is assigned to this one
  *\return A reference to this object after the assignment.
  */
  Vector3<V>& operator=(const Vector3<V>& other) 
  {
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
  }

  /** Copy constructor
  *\param other The other vector that is copied to this one
  */
  Vector3<V>(const Vector3<V>& other){*this = other;}

  /** Addition of another vector to this one. 
  *\param other The other vector that will be added to this one
  *\return A reference to this object after the calculation.
  */
  Vector3<V>& operator+=(const Vector3<V>& other)
  {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  /** Substraction of this vector from another one.
  *\param other The other vector this one will be substracted from 
  *\return A reference to this object after the calculation.
  */
  Vector3<V>& operator-=(const Vector3<V>& other)
  {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  /** Multiplication of this vector by a factor.
  *\param factor The factor this vector is multiplied by 
  *\return A reference to this object after the calculation.
  */
  Vector3<V>& operator*=(const V& factor)
  {
    x *= factor;
    y *= factor;
    z *= factor;
    return *this;
  }

  /** Division of this vector by a factor.
  *\param factor The factor this vector is divided by 
  *\return A reference to this object after the calculation.
  */
  Vector3<V>& operator/=(const V& factor)
  {
    if (factor == 0) return *this;
    x /= factor;
    y /= factor;
    z /= factor;
    return *this;
  }

  /** Addition of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A new object that contains the result of the calculation.
  */
  Vector3<V> operator+(const Vector3<V>& other) const
    {return Vector3<V>(*this) += other;}

  /** Subtraction of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A new object that contains the result of the calculation.
  */
  Vector3<V> operator-(const Vector3<V>& other) const
    {return Vector3<V>(*this) -= other;}

  /** Inner product of this vector and another one.
  *\param other The other vector this one will be multiplied by 
  *\return The inner product.
  */
  V operator*(const Vector3<V>& other) const
  {
    return (x*other.x + y*other.y + z*other.z);
  }

  /** Multiplication of this vector by a factor.
  *\param factor The factor this vector is multiplied by 
  *\return A new object that contains the result of the calculation.
  */
  Vector3<V> operator*(const V& factor) const
    {return Vector3<V>(*this) *= factor;}

  /** Division of this vector by a factor.
  *
  *\param factor The factor this vector is divided by 
  *\return A new object that contains the result of the calculation.
  */
  Vector3<V> operator/(const V& factor) const
    {return Vector3<V>(*this) /= factor;}

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are equal.
  */
  bool operator==(const Vector3<V>& other) const
  {
    return (x==other.x && y==other.y && z==other.z);
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are unequal.
  */
  bool operator!=(const Vector3<V>& other) const
    {return !(*this == other);}

  
  /**
  * array-like member access.
  * \param i index of coordinate
  * \return reference to x, y or z
  */
  V& operator[](int i)
  {
    return  (&x)[i];
  }

  const V& operator[](int i) const
  {
    return  (&x)[i];
  }
  
  /** Calculation of the length of this vector.
  *\return The length.
  */
  V abs() const 
  {return (V) sqrt(double((x*x) + (y*y) + (z*z)));}

  /** Calculation of the square length of this vector.
  *\return length*length.
  */
  V squareAbs() const
  {return (x*x) + (y*y) + (z*z);}

  /** Crossproduct of this vector and another vector.
  *\param other The factor this vector is multiplied with.
  *\return A new object that contains the result of the calculation.
  */
  Vector3<V> operator^(const Vector3<V>& other) const
    {return Vector3<V>(y * other.z - z * other.y, 
                    z * other.x - x * other.z, 
                    x * other.y - y * other.x);}

  /** Crossproduct of this vector and another vector.
  *\param other The factor this vector is multiplied with.
  *\return A reference to this object after the calculation.
  */
  Vector3<V>& operator^=(const Vector3<V>& other)
    {*this = *this ^ other; return *this;}

	/** normalize this vector.
  *\param len The length, the vector should be normalized to, default=1.
  *\return the normalized vector.
  */
  Vector3<V> normalize(V len)
  {
    V lenghtOfVector = abs();
    if (lenghtOfVector == 0) return *this;
    return *this = (*this * len) / lenghtOfVector;
   }

 /** normalize this vector.
  *\return the normalized vector.
  */
  Vector3<V> normalize()
  {
    V lenghtOfVector = abs();
    if (lenghtOfVector == 0) return *this;
    return *this /= lenghtOfVector;
  }
    
    /** Negation of this vector.
     *\return A new object that contains the result of the calculation.
     */
    inline Vector3<V> operator-() const
    {
        return Vector3<V>(-x, -y, -z);
    }

    /**
    * @brief output stream operator result of the form (x,y,z)
    */
    friend std::ostream& operator<< (std::ostream& output, const Vector3<V>& v)
    {
        output << "(" << v.x << "," << v.y << "," << v.z << ")";
        return output;
    }

    /**
    * @brief input stream operator for the form (x,y,z)
    */
    friend std::istream& operator>>(std::istream& stream, Vector3<V>& v)
    {
        stream.ignore(2,'(');
        stream >> v.x;
        stream.ignore(2,',');
        stream >> v.y;
        stream.ignore(2,',');
        stream >> v.z;
        stream.ignore(2,')');
        stream.peek();
        return stream;
    }
};

#endif // __Vector3_h__

