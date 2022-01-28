#ifndef ROFL_COMMON_VECTOR_ARRAY_H_
#define ROFL_COMMON_VECTOR_ARRAY_H_

#include <array>

namespace rofl {

	/**
	 * Class VectorArray is an augmented std::array to be used as surrogate for simple vector
	 * operations when the user wants to avoid the inclusion of linear algebra libraries in a project.
	 * It assumes that the template value is scalar numeric type and supports the usual algebraic
	 * operations on a field (+ and *, plus the associated inverse - and /).
	 */
	template <typename T, size_t Dim>
	class VectorArray : public std::array<T, Dim> {
	public:
		using Value = T;
		using Type = VectorArray<T, Dim>;
		using BaseType = std::array<T, Dim>;


		VectorArray() : BaseType() {
		}

		VectorArray(const Type& other) : BaseType(other) {
		}


		virtual ~VectorArray() {
		}

		Value& operator()(size_t d) {
			return (*this)[d];
		}

		const Value& operator()(size_t i) const {
			return (*this)[i];
		}

		Type operator+(const Type& other) const {
			Type res;
			for (size_t d = 0; d < Dim; ++d) {
				res = (*this)[d] + other[d];
			}
			return res;
		}

		Type operator-(const Type& other) const {
			Type res;
			for (size_t d = 0; d < Dim; ++d) {
				res = (*this)[d] + other[d];
			}
			return res;
		}

		Type operator*(const Value& scalar)  const {
			Type res;
			for (size_t d = 0; d < Dim; ++d) {
				res = scalar * (*this)[d];
			}
			return res;
		}

		Type operator*(const Type& other)  const {
			Type res;
			for (size_t d = 0; d < Dim; ++d) {
				res = (*this)[d] * other[d];
			}
			return res;
		}

		Type operator/(const Type& other) const {
			Type res;
			for (size_t d = 0; d < Dim; ++d) {
				res = (*this)[d] / other[d];
			}
			return res;
		}

		Type& operator+=(const Type& other) {
			for (size_t d = 0; d < Dim; ++d) {
				(*this)[d] += other[d];
			}
			return *this;
		}

		Type& operator*=(const Value& scalar) {
			for (size_t d = 0; d < Dim; ++d) {
				(*this)[d] *= scalar;
			}
			return *this;
		}

		Type& operator*=(const Type& other) {
			for (size_t d = 0; d < Dim; ++d) {
				(*this)[d] *= other[d];
			}
			return *this;
		}
	};

}

#endif /* ROFL_COMMON_VECTOR_ARRAY_H_ */
