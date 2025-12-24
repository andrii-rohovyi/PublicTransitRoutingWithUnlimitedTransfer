#pragma once

#include <vector>
#include <algorithm>
#include <utility> // For std::forward

/**
 * @namespace Vector
 * @brief Provides utility functions for std::vector and vector-like containers.
 */
namespace Vector {

    /**
     * @brief Fills an entire vector or vector-like container with a specified value.
     *
     * This is a utility wrapper for std::fill that is cleaner to use 
     * in the context of a dedicated utilities namespace.
     * * @tparam Container The type of the container (e.g., std::vector<T>).
     * @tparam Value The type of the value to fill with.
     * @param container The container to be filled.
     * @param value The value to assign to all elements.
     */
    template<typename Container, typename Value>
    inline void fill(Container& container, const Value& value) noexcept {
        // Use std::fill with the container's iterators
        std::fill(container.begin(), container.end(), value);
    }
    
    /**
     * @brief Resets all elements in a vector to their default-constructed state.
     * * This is often faster than iterating and reassigning for basic types,
     * but relies on the element type having a cheap default constructor.
     * * @tparam T The element type of the vector.
     * @param vector The std::vector to be reset.
     */
    template<typename T>
    inline void resetToDefault(std::vector<T>& vector) noexcept {
        // Re-initializes all elements by moving a default-constructed vector over,
        // which can be optimized by the compiler to just clear the memory 
        // for primitive types.
        std::vector<T> empty(vector.size());
        vector.swap(empty);
    }

    // A simple wrapper for reversing the order of elements in a container.
    template<typename Container>
    inline Container reverse(Container container) noexcept {
        std::reverse(container.begin(), container.end());
        return container;
    }

} // namespace Vector