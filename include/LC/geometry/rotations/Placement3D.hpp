
#pragma once

#include <LC/geometry/rotations/Placement3D.hpp>

namespace lc
{
	// ================================================================
	//                          3D PLACEMENT
	// ================================================================

	template <class T = double>
	class Placement3
	{
	public:
		// ================================
		//          CONSTRUCTORS
		// ================================

		// default constructor: position and rotation are set to 0, scale is set to 1
		constexpr Placement3() = default;

		// move constructor
		constexpr Placement3(Placement3&&) = default;

		// copy constructor
		constexpr Placement3(const Placement3&) = default;

		// ================================
		//           OPERATIONS
		// ================================

		// this function takes a vector, returns a transformed vector:
		// v -> scale -> rotate -> translate -> result
		constexpr Vector3<T> apply_to(const Vector3<T>& v) const;

		// ================================
		//             ACCESS
		// ================================

		// TODO
		// Placement3<T> relative_to

		// returns a placement that is equivalent to this but as child on parent:
		// this -> scale by parent (rel 2 O) -> rotate by parent (rel 2 O) -> translate by parent -> result
		constexpr Placement3<T> as_child_of(const Placement3<T>& parent) const;

		Mat<4, 4, T> getMatrix(void) const;

		Placement3<T> inverse(void) const;

		// ================================
		//           OPERATORS
		// ================================

		constexpr Placement3& operator=(const Placement3&) = default;
		constexpr Placement3& operator=(Placement3&&) = default;

	public:

		// ================================
		//             DATA
		// ================================

		// relative translation of the object
		Vector3<T> translation_vec;

		// relative scale of the object
		T scale_factor = T(1);

		// relative rotation of the object
		Rotation3<T> rotation_value;
	};

	using Placement3d = Placement3<double>;

#if __has_include(<cereal/cereal.hpp>)
	// TODO move and template
	template <class Archive>
	void serialize(Archive& archive, Placement3d& placement, const Uint32 version);
#endif
}

// ================================================================================================================================
// ================================================================================================================================
//                                                          INL
// ================================================================================================================================
// ================================================================================================================================

namespace lc
{
	// ================================================================
	//                          3D PLACEMENT
	// ================================================================

	// ================================
	//          CONSTRUCTORS
	// ================================

	// ================================
	//           OPERATIONS
	// ================================

	////////////////////////////////////////////////////////////////
	template <class T>
	constexpr Vector3<T> Placement3<T>::apply_to(const Vector3<T>& v) const
	{
		auto result = v;

		// scale the vector
		//          ^
		// ^   =>   |    
		// |        |
		result *= this->scale_factor;

		// rotate the vector
		//          ^         ^
		// ^   =>   |   =>   /
		// |        |       /
		result = this->rotation_value.rotate_vector(result);

		// translate the vector        ^
		//          ^         ^       /
		// ^   =>   |   =>   /   =>  /
		// |        |       /          .
		result += this->translation_vec;

		return result;
	}

	// ================================
	//             ACCESS
	// ================================

	////////////////////////////////////////////////////////////////
	template <class T>
	inline constexpr Placement3<T> Placement3<T>::as_child_of(const Placement3<T>& parent) const
	{
		// ...
		auto tmp = *this;

		// scale (relative to origin)
		tmp.scale_factor *= parent.scale_factor;
		tmp.translation_vec *= parent.scale_factor;

		// rotate (relative to origin)
		tmp.rotation_value *= parent.rotation_value;
		tmp.translation_vec = parent.rotation_value.rotate_vector(tmp.translation_vec);

		// translate
		tmp.translation_vec += parent.translation_vec;

		return tmp;
	}

	////////////////////////////////////////////////////////////////
	template <class T>
	inline Mat<4, 4, T> Placement3<T>::getMatrix(void) const
	{
		auto m = rotation_value.to_Mat3x3();

		// TODO cavolo manca!!!!!
		//m *= scale_factor;
		// (solo la sottomatrice in alto a sinistra 3x3)
		for (size_t i = 0; i < 3; i++)
			for (size_t j = 0; j < 3; j++)
				m.at(i, j) *= scale_factor;
		
		Mat<4, 4, T> M;

		for (size_t i = 0; i < 3; i++)
			for (size_t j = 0; j < 3; j++)
				M.at(i, j) = m.at(i, j);

		for (size_t i = 0; i < 3; i++)
			M.at(i, 3) = translation_vec[i];

		M.at(3, 3) = T(1);

		return M;
	}

	////////////////////////////////////////////////////////////////
	template <class T>
	inline Placement3<T> Placement3<T>::inverse(void) const
	{
		// TODO_IMPORTANT to verify correctness
		Placement3<T> translation; translation.translation_vec = -this->translation_vec;
		Placement3<T> rotation; rotation.rotation_value = -this->rotation_value;
		Placement3<T> scale; scale.scale_factor = T(1) / this->scale_factor;
		return translation.as_child_of(rotation.as_child_of(scale));
	}

#if __has_include(<cereal/cereal.hpp>)
	// TODO move
	template <class Archive>
	void serialize(Archive& archive, Placement3d& placement, const Uint32 version)
	{
		if (version == 0)
		{
			archive(cereal::make_nvp("translation_vec", placement.translation_vec));
			archive(cereal::make_nvp("scale_factor", placement.scale_factor));
			archive(cereal::make_nvp("rotation_value", placement.rotation_value));
			return;
		}

		assert(0);
	}
#endif
}

#if __has_include(<cereal/cereal.hpp>)
CEREAL_CLASS_VERSION(lc::Placement3d, 0);
#endif


