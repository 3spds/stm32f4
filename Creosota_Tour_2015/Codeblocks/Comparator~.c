
#include "m_pd.h"
#include "math.h"
#ifdef NT
#pragma warning( disable : 4244 )
#pragma warning( disable : 4305 )
#endif

/* ------------------------ comparator~ ----------------------------- */



static t_class *comparator_class;

typedef struct _comparator
{
    t_object x_obj; 	/* obligatory header */
    t_float x_f;    	/* place to hold inlet's value if it's set by message */

} t_comparator;


t_int *comparator_perform(t_int *w)
{
    t_float *in1 = (t_float *)(w[1]);
    t_float *in2 = (t_float *)(w[2]);
    t_float *out = (t_float *)(w[3]);
    int n = (int)(w[5]);

    while (n--)
    {
    	t_float pos = *(in1++); //grab input vector #1
    	t_float neg = *(in2++); //grab input vector 2
    	*out = (pos > neg ? (1) : (0))
    }
    return (w+5);
}

static void comparator_dsp(t_comparator *x, t_signal **sp)
{
    dsp_add(comparator_perform, 4, sp[0]->s_vec, sp[1]->s_vec, sp[2]->s_vec, sp[0]->s_n);
}


static void *comparator_new(void)
{
        t_comparator *x = (t_comparator *)pd_new(comparator_class);
        inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("signal"), gensym("signal"));
        outlet_new(&x->x_obj, gensym("signal"));
        x->x_f = 0;
        return (x);
}

void comparator_tilde_setup(void)
{
    comparator_class = class_new(gensym("comparator~"), (t_newmethod)comparator_new, 0,
    	sizeof(t_comparator), 0, A_GIMME, 0);

    CLASS_MAINSIGNALIN(comparator_class, t_comparator, x_f);

    class_addmethod(comparator_class, (t_method)comparator_dsp, gensym("dsp"), 0);
}
