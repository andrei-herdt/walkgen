#ifndef QP_WRAPPER_INL_H_
#define QP_WRAPPER_INL_H_

public:
//\name Accessors and mutators
//\{
inline QPMatrix &hessian_mat() 			{return hessian_mat_;}
inline QPVector &objective_vec() 		{return objective_vec_;};

inline QPMatrix &constr_mat() 			{return constr_mat_;}
inline QPVector &uc_bounds_vec() 		{return uc_bounds_vec_;};
inline QPVector &lc_bounds_vec() 		{return lc_bounds_vec_;};
inline QPVector &uv_bounds_vec() 		{return uv_bounds_vec_;};
inline QPVector &lv_bounds_vec() 		{return lv_bounds_vec_;};

inline void num_var (int num_var) 			{assert(num_var <= num_var_max_); num_var_ = num_var;}
inline int num_var () const 				{return num_var_;}

inline void num_var_max (int num_var) 		{num_var_max_ = num_var;}
inline int num_var_max () const 			{return num_var_max_;}

inline void num_constr(int num_constr) 		{assert(num_constr <= num_constr_max_); num_constr_ = num_constr;}
inline int num_constr() const 				{return num_constr_;}

inline void num_eq_constr(int num_eq_constr) 		{num_eq_constr_ = num_eq_constr;}
inline int num_eq_constr() const 				{return num_eq_constr_;}

inline void num_constr_max(int num_constr) 	{num_constr_max_ = num_constr;}
inline int num_constr_max() const 			{return num_constr_max_;}
//\}


#endif /* QP_WRAPPER_INL_H_ */
