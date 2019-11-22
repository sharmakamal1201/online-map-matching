import math
#from scipy import integrate
#import sympy as sp
from scipy.integrate import dblquad

'''
formula: p(T) = product_over_i( p[e(i) -> e(i+1)] )
			p(r(k+1)|r(k),T) = int{ p_x * int{ p_v * p(r(k+1)|r(k),T,x,v) } }
'''
def prediction(curr_road,en):
	sigma_v = 25
	vm = 30
	delta_t = 20
	#rk_l = road_length(curr_road)
	#p_T = Trans_matrix[curr_road][en]
	#T_1_to_n_l = road_length(en)
	#T_1_to_n_minus_1_l = road_length(curr_road)
	T_1_to_n_minus_1_l = 20
	T_1_to_n_l = 30
	rk_l = 10

	def p_v(v):
		return (1/(math.sqrt(2*math.pi)*sigma_v))*math.exp(-0.5*((v-vm)/sigma_v)*((v-vm)/sigma_v))
	def p_x():
		return (1/rk_l)
	def ll(x):
		return (T_1_to_n_minus_1_l - x)/delta_t
	def ul(x):
		return (T_1_to_n_l - x)/delta_t

	area = dblquad(lambda v, x: (p_x())*(p_v(v)), 0, rk_l, lambda v: ll(v), lambda v: ul(v))
	print(area[0])



def main():
	prediction(0,1)

if __name__ == "__main__":
	main()