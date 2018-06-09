# test
# include("function_test.jl")
# tic()
# haha(3)
# toc()
a = rand(4000,5)
tic()
b = copy(a)
toc()
# tic()
# b = a.^2
# sum(b,2)
# toc()

# tic()
# b = a.^2
# c = zeros(size(b,1),1)
# for i in size(a,2)
# 	c+=b[:,i]
# end
# toc()

# tic()
# b = a.^2 
# b[:,1]+b[:,2]+b[:,3]+b[:,4]+b[:,5]
# toc()