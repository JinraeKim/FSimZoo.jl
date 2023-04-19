"""
Moore-Penrose inverse control allocator.
"""
struct PseudoInverseAllocator <: StaticAllocator
    B_pinv
    function PseudoInverseAllocator(B)
        B_pinv = pinv(B)
        new(B_pinv)
    end
end

"""
# Variables
ν: virtual input
# Notes
ν = B*u where u: control input
"""
function Command(allocator::PseudoInverseAllocator)
    (; B_pinv) = allocator
    return function (ν, Λ=Diagonal(ones(size(B_pinv)[1])))
        (pinv(Λ) * B_pinv) * ν  # pinv(B*Λ) = pinv(Λ) * pinv(B)
    end
end

