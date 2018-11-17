#include <iostream>
#include <vector>

int main(){
	std::cout<<"Hello"<<std::endl;
	std::vector<int> nums = {1,2,3,4,5};

	for(int i = 0; i<nums.size(); i++){
		std::cout<<nums[i]<<std::endl;
	}
	return 1;
}