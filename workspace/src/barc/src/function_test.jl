__precompile__()
module haha
export Foo
type Foo
	a
	b
	function Foo()
		n = new()
		n.a = 1
		n.b = 2
		return n
	end
end
end

# type F
# 	a
# 	f
# 	function F(foo::Foo)
# 		f=new()
# 		f.a = 1
# 		f.f = foo
# 		return f
# 	end
# end

# function haha(a::Int64)
# 	println("aaaaa")
# end

# function hahaha(a::Int64)
# 	if a>1
# 		haha(a)
# 	end
# 	println("bbbbb")
# end