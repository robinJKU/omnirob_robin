#ifndef __COMMON_CPP_TOOLS_H
#define __COMMON_CPP_TOOLS_H

namespace common_cpp_tools{
	/**
	 * Converts a number to a string.
	 */
	std::string num_to_strin( int number ){
		std::ostringstream convert;
		convert << number;

		return convert.str();
	}
	/**
	 * Returns the first appearance of the token in the array or -1 if the token isn't found.
	 * @return index of token in array or -1
	 */
	inline int find_string( std::vector<std::string> array, std::string token){
		int pos = std::find(array.begin(), array.end(), token) - array.begin();
		return pos<array.size()? pos: -1;
	}
	/**
	 * Returns the minimum value of the elements of array
	 * @return min value or 0 if the array is empty
	 */
	int find_min_value( std::vector<int> array ){
		if( array.size()<1 ){
			return 0;
		}
		int min_value = array[0];
		for( unsigned int ii=1; ii<array.size(); ii++ ){
			if( array[ii]<min_value ){
				min_value = array[ii];
			}
		}
		return min_value;
	}


}
#endif
