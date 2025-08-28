#include <iostream>

// Forward declarations for test functions
void test_ray_functionality();
void test_triangle_functionality();
void test_intersect_functionality();

int main() {
    std::cout << "Running geometry library tests...\n\n";
    
    try {
        test_ray_functionality();
        test_triangle_functionality();  
        test_intersect_functionality();
        
        std::cout << "\n🎉 All tests passed!\n";
        return 0;
    } catch (const std::exception& e) {
        std::cout << "\n❌ Test failed with exception: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cout << "\n❌ Test failed with unknown exception\n";
        return 1;
    }
}
