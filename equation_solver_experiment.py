import sympy
import time

start_time = time.time()


x = sympy.symbols('x')
#expr = x-4-2

b=2.0
a=1.5
h=1.1

expr = b*sympy.sqrt(x)+(2/b)*sympy.sqrt(x)*x-a
#sol = sympy.solve(expr)
sol =sympy.solveset(expr, domain=sympy.S.Reals)

#expr2 = sympy.simplify(expr)

print(sol)

myx = list(sol)[0]
print(myx)


#expr3 = (a-x)**2+(b-h*sympy.sqrt(x))**2
#-4.4*sqrt(x) + 1.0*x**2 - 1.79*x + 6.25
#diff3 = sympy.diff(expr3)

#print(diff3)

#sol =sympy.solveset(diff3, domain=sympy.S.Reals)
#print(sol)

#print(sympy.simplify(expr3))

print("Duration:", time.time()-start_time)
