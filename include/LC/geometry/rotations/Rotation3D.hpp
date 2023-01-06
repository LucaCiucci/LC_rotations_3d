#pragma once

// TODO su arithmetics iniseme a tracer
#include <LC/hyperxomplex/Quaternion.hpp>
#include <LC/math/linalg/Matrix/Matrix.hpp>

#if __has_include(<cereal/cereal.hpp>)
    #include <cereal/cereal.hpp>
#endif

namespace lc
{
	// ================================================================
	//                           ROTATION3
	// ================================================================

	// this class is aimed to represent 3d spatial rotations
	template <lc::math::concepts::ScalarType T = double>
	class Rotation3 : private Vector3<T>
	{
		// TODO BASE a' occupato!!!!!!!
		using BASE_TYPE = Vector3<T>;

	public:

		static constexpr Rotation3<T> from_euler(T yaw, T pitch, T roll);
		static constexpr Rotation3<T> between_dirs(const Vector3<T>& dir_from, const Vector3<T>& dir_to);


		// ================================
		//          CONSTRUCTORS
		// ================================

		// default constructor
		constexpr Rotation3() = default;

		// copy constructor
		constexpr Rotation3(const Rotation3<T>&) = default;

		// copy constructor
		constexpr Rotation3(Rotation3<T>&&) = default;

		// copy constructor from different underlying type
		template <class T2>
		constexpr Rotation3(const Rotation3<T2>& other);

		// construct from components
		constexpr Rotation3(T rx, T ry, T rz);

		// construct from vector
		explicit constexpr Rotation3(const Vector3<T>& rot_vec);

		// construct from rotation quaternion
		explicit constexpr Rotation3(Quaternion<T> rot_q);

		// construct from dir and angle
		explicit constexpr Rotation3(const Vector3<T>& dir, T angle);

		// ================================
		//             ACCESS
		// ================================

		// expose the components:
		//using BASE_TYPE::X;
		using BASE_TYPE::x;
		using BASE_TYPE::y;
		using BASE_TYPE::z;

		// access and iterators
		using BASE_TYPE::size;
		using BASE_TYPE::begin;
		using BASE_TYPE::cbegin;
		using BASE_TYPE::end;
		using BASE_TYPE::cend;
		using BASE_TYPE::front;
		using BASE_TYPE::back;
		using BASE_TYPE::at;
		using BASE_TYPE::operator[];
		using BASE_TYPE::data;

		// ================================
		//           CONVERSIONS
		// ================================

		// as vector
		constexpr Vector3<T>& vec(void);

		// as vector
		constexpr const Vector3<T>& vec(void) const;

		// convert to quaternion
		constexpr Quaternion<T> to_quaternion(void) const;

		// convert to a rotation matrix
		constexpr Mat<3, 3, T> to_Mat3x3(void) const;

		// rotation angle
		T angle(void) const;

		// rotation dir
		Vector3<T> dir(void) const;

		// ================================
		//           OPERANTIONS
		// ================================

		// apply this rotation to a vector
		constexpr Vector3<T> rotate(const Vector3<T>& vec) const;

		// invert the rotation (change sign)
		constexpr Rotation3<T>& invert(void);

		// get the inverse rotation (opposite sign)
		constexpr Rotation3<T> inverse(void) const;

		// ================================
		//            OPERATORS
		// ================================

		// copy assignment
		constexpr Rotation3<T>& operator=(const Rotation3<T>&) = default;

		// move assignment
		constexpr Rotation3<T>& operator=(Rotation3<T>&&) = default;

		// get the inverse rotation
		constexpr Rotation3<T> operator-(void) const;

		// compose the rotation with another:
		// this * other = q1 * q2
		// in quaternioni: this * other significa in realt� q2 * q1 perce'
		// cosi' e' q2 q1 p q1' q2'
		constexpr Rotation3<T>& operator*=(const Rotation3<T>&right);

		// compose the rotation with another:
		// this * other = q1 * q2
		// in quaternioni: this * other significa in realt� q2 * q1 perce'
		// cosi' e' q2 q1 p q1' q2'
		constexpr Rotation3<T> operator*(const Rotation3<T>&right) const;

		// ================================
		//          SERIALIZATION
		// ================================

#if __has_include(<cereal/cereal.hpp>)
		template <class Archive>
		void serialize(Archive& archive, const Uint32 version);
#endif

	private:


	};

	using Rotation3d = Rotation3<double>;
}

namespace lc
{
	namespace math
	{
		namespace geometry
		{
			using lc::Rotation3;
			using lc::Rotation3d;
		}
		using lc::Rotation3;
		using lc::Rotation3d;
	}
}

// ================================================================
//                       external functions
// ================================================================

// ================================
//              I/O
// ================================

// formatted output
template <class T>
std::ostream& operator<<(std::ostream& os, const lc::Rotation3<T>& rot);

namespace lc
{
	// formatted srting
	template <class T>
	std::string to_string(const lc::Rotation3<T>& rot);
}

// ================================================================================================================================
// ================================================================================================================================
//                                                         INL
// ================================================================================================================================
// ================================================================================================================================

namespace lc
{
	// ================================================================
	//                           ROTATION3
	// ================================================================

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Rotation3<T> Rotation3<T>::from_euler(T yaw, T pitch, T roll)
	{
		Rotation3<T> tmp    (Vector3<T>( (T)0, (T)0, roll));
		tmp *= Rotation3<T>( Vector3<T>(pitch, (T)0, (T)0));
		tmp *= Rotation3<T>( Vector3<T>( (T)0,  yaw, (T)0));
		return tmp;
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Rotation3<T> Rotation3<T>::between_dirs(const Vector3<T>& dir_from, const Vector3<T>& dir_to)
	{
		auto r = dir_from.cross(dir_to);
		auto p = dir_from.dot(dir_to);
		auto p_n = dir_from.norm() * dir_to.norm();

		// TODO il caso in cui e' 180 e r fa zero, allora bisogna fare una scelta arbitraria...

		// TODO meglio per trattare tutti i casi di differenziabilit�

		r.setNorm(dir_from.ang(dir_to));

		return r;
	}

	// ================================
	//          CONSTRUCTORS
	// ================================

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	template <class T2>
	inline constexpr Rotation3<T>::Rotation3(const Rotation3<T2>& other) :
		Vector3<T>((T)other.x(), (T)other.y(), (T)other.z())
	{
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Rotation3<T>::Rotation3(T rx, T ry, T rz) :
		// init components
		Vector3<T>(rx, ry, rz)
	{
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Rotation3<T>::Rotation3(const Vector3<T>& rot_vec) :
		// init components
		Vector3<T>(rot_vec)
	{
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Rotation3<T>::Rotation3(Quaternion<T> rot_q) :
		Rotation3()
	{
		// quaternion:
		//     q = cos(teta/2) + sin(teta/2){bi + cj + dk} = cos(teta/2) + sin(teta/2) * n
		// we need the normal vector v and the angle tetha

		// at first, we ensure the quaternion is normalized:
		{
			auto norm = rot_q.norm();
			if (norm <= 0)
				return;// we consider the rotation as zero, null
			else
				rot_q.normalize();
		}

		// now, we get the rotation vector (imaginary part)
		auto v = rot_q.get_vec3();

		// we store the norm of the imaginary part
		auto sin_teta_2 = v.norm();
		auto cos_teta_2 = rot_q.a;

		if (sin_teta_2 == 0)
			return;// rotation is null

		// we can now calculate teta
		//auto teta = asin(sin_teta_2) * T(2);
		auto teta = acos(cos_teta_2) * T(2);

		if (abs(cos_teta_2 - T(1)) < 0.1)
			teta = asin(sin_teta_2) * T(2);

		// calculate the rotation versor:
		auto n = v.versor();

		// finally, we define the rotation vector as:
		Vector3<T> rot_vec = n * teta;

		(Vector3<T>&)* this = rot_vec;
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Rotation3<T>::Rotation3(const Vector3<T>& dir, T angle) :
		Rotation3(dir.versor() * angle)// TODO con set_norm -> with_norm
	{

	}

	// ================================
	//             ACCESS
	// ================================

	// ================================
	//           CONVERSIONS
	// ================================

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Vector3<T>& Rotation3<T>::vec(void)
	{
		return *this;
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr const Vector3<T>& Rotation3<T>::vec(void) const
	{
		return *this;
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Quaternion<T> Rotation3<T>::to_quaternion(void) const
	{
		/*
		* intuitive version:
		* 
		*  // calculate direction
		*  Vector3<T> n = this->versor();
		*  
		*  // rotation angle
		*  T theta_2 = this->norm() / T(2);
		*  
		*  // scale
		*  n *= sin(theta_2);
		*  
		*  return Quaternion<T>(n, cos(theta_2));
		*/

		// version without singularities ...
		auto norm_this = this->norm();

		T theta_2 = norm_this / T(2);

		// v = this / norm * sin(theta_2)
		//   = this / (theta_2 * 2) * sin(theta_2)
		//   = this/2 * (sin(theta_2) / theta_2)

		Vector3<T> v = *this;
		v *= priv::sinx_over_x<T>(theta_2, T(1e-6)) / T(2); // !!!!!

		// In this way there is a still a singularity in 0, but the problem is managed by the sinx_over_x()
		// function. This also warks for differentials which would certainly calculate some wrong
		// derivatives in sqrt() and the division, but the function sinx_over_x() would evaluate to 1
		// and the correct derivatives will be correctly propagated

		return Quaternion<T>(v, cos(theta_2));
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Mat<3, 3, T> Rotation3<T>::to_Mat3x3(void) const
	{
		Mat<3, 3, T> R;
		for (size_t j = 0; j < 3; j++)
		{
			Vector3<T> e_j; e_j[j] = T(1);// TODO penso che i, j, k e e_i siano gia' definiti

			auto r = this->rotate(e_j);

			for (size_t i = 0; i < 3; i++)
				R[i][j] = r[i];
		}

		return R;
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline T Rotation3<T>::angle(void) const
	{
		return this->norm();
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline Vector3<T> Rotation3<T>::dir(void) const
	{
		auto v = this->vec();
		if (v == T(0))
			return v;
		else
			return v.versor();
	}

	// ================================
	//           OPERANTIONS
	// ================================

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Vector3<T> Rotation3<T>::rotate(const Vector3<T>& vec) const
	{
		auto q = this->to_quaternion();

		Quaternion<T> p(vec);

		p =  q * p * q.inverse();

		return p.get_vec3();// TODO con funzione!!!!!!
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Rotation3<T>& Rotation3<T>::invert(void)
	{
		Vector3<T>::flip();
		return *this;
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Rotation3<T> Rotation3<T>::inverse(void) const
	{
		auto tmp = *this;
		tmp.invert();
		return tmp;
	}

	// ================================
	//            OPERATORS
	// ================================

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Rotation3<T> Rotation3<T>::operator-(void) const
	{
		return this->inverse();
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Rotation3<T>& Rotation3<T>::operator*=(const Rotation3<T>& right)
	{
		// TODO da ricontrollare, forse c'� un metodo pi� intelligente che non passa per questi quaternioni

		auto q1 = this->to_quaternion();
		auto q2 = right.to_quaternion();

		auto q_tot = q2 * q1;

		*this = (Rotation3<T>)q_tot;

		return *this;
	}

	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	inline constexpr Rotation3<T> Rotation3<T>::operator*(const Rotation3<T>& right) const
	{
		auto tmp = *this;
		tmp *= right;// sbalgiato????????
		return tmp;
	}

	// ================================
	//          SERIALIZATION
	// ================================

#if __has_include(<cereal/cereal.hpp>)
	////////////////////////////////////////////////////////////////
	template <lc::math::concepts::ScalarType T>
	template <class Archive>
	inline void Rotation3<T>::serialize(Archive& archive, const Uint32 version)
	{
		if (version == 0)
		{
			archive(cereal::make_nvp("x", this->x()));
			archive(cereal::make_nvp("y", this->y()));
			archive(cereal::make_nvp("z", this->z()));

			return;
		}

		// TODO exception
		assert(0);
	}
#endif
}

#if __has_include(<cereal/cereal.hpp>)
// see https ://github.com/USCiLab/cereal/issues/319
// CEREAL_CLASS_VERSION(Vector, 0);
namespace cereal {
	namespace detail {
		template <class T>
		struct Version<::lc::Rotation3<T>>
		{
			static const std::uint32_t version;
			static std::uint32_t registerVersion()
			{
				constexpr std::uint32_t v = 0; // VERSION!
				::cereal::detail::StaticObject<Versions>::getInstance().mapping.emplace(
					std::type_index(typeid(::lc::Rotation3<T>)).hash_code(), v);
				return v;
			}
			static void unused() { (void)version; }
		}; /* end Version */
		template <class T>
		const std::uint32_t Version<::lc::Rotation3<T>>::version =
			Version<::lc::Rotation3<T>>::registerVersion();
	}
} // end namespaces
#endif

// ================================================================
//                       external functions
// ================================================================

// ================================
//              I/O
// ================================

#include <iostream>

////////////////////////////////////////////////////////////////
template <class T>
inline std::ostream& operator<<(std::ostream& os, const lc::Rotation3<T>& rot)
{
	using namespace lc;

	//const Vector3<T>& rot_vec = (const Vector3<T>&)rot;

	os << "R(";

	for (auto it = rot.begin(); it != rot.end(); it++)
	{
		os << *it;
		if (it == rot.end() - 1)
			os << ")";
		else
			os << ", ";
	}

	return os;
}

namespace lc
{
	////////////////////////////////////////////////////////////////
	template <class T>
	inline std::string to_string(const lc::Rotation3<T>& rot)
	{
		std::stringstream ss;
		ss << rot;
		return ss.str();
	}
}