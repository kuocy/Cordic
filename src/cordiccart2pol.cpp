#include "cordiccart2pol.h"

float cordic_ang[16] =
{
		45.00000000,
		26.56505118,
		14.03624347,
		7.125016349,
		3.576334375,
		1.789910608,
		0.8951737102,
		0.4476141709,
		0.2238105004,
		0.1119056771,
		0.05595289189,
		0.02797645262,
		0.01398822714,
		0.006994113675,
		0.003497056851,
		0.001748528427
};


void cordiccart2pol(data_t x, data_t y, data_t * r,  data_t * theta)
{
//#pragma HLS INTERFACE m_axi port=r offset=slave depth=1
//#pragma HLS INTERFACE m_axi port=theta offset=slave depth=1
#pragma HLS INTERFACE s_axilite port=r bundle=CTRL
#pragma HLS INTERFACE s_axilite port=theta bundle=CTRL

#pragma HLS INTERFACE s_axilite port=x bundle=CTRL
#pragma HLS INTERFACE s_axilite port=y bundle=CTRL
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL

	ap_fixed<32, 2, AP_RND, AP_WRAP, 1> current_x = x;
	ap_fixed<32, 2, AP_RND, AP_WRAP, 1> current_y = y;

	data_t spin, temp_spin;

    //step 1
	if (y >= 0)
    {
        current_x = y;
        current_y = -x;
        //spin = 90;
        temp_spin = 90;
    }
    else
    {
        current_x = -y;
        current_y = x;
        //spin = -90;
        temp_spin = -90;
    }

    //step2

	#pragma HLS PIPELINE II = 1
    for (int i = 0; i < NO_ITER; i++)
    {
		//#pragma HLS LOOP_TRIPCOUNT min=1 max=16
    	ap_fixed<32, 2, AP_RND, AP_WRAP, 1> shifter_x = current_x >> i;
    	ap_fixed<32, 2, AP_RND, AP_WRAP, 1> shifter_y = current_y >> i;

    	if (current_y >= 0)
    	{
    		current_x = current_x + shifter_y;
    		current_y = current_y - shifter_x;
    		spin = temp_spin + cordic_ang[i];
    	}
    	else
    	{
    		current_x = current_x - shifter_y;
    		current_y = current_y + shifter_x;
    		spin = temp_spin - cordic_ang[i];
    	}

    	temp_spin = spin;

    }

	//final
    *r = float(current_x) * 0.607253;
    *theta = float(spin) * 2 * 3.1415926 / 360;
    //*r = current_x;
    //*theta = spin;


}
