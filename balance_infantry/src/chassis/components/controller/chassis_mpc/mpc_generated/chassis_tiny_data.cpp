// Generated fixed-model cache; official ADMM runs online.
#include "chassis_tiny_data.hpp"
#include "chassis_mpc_config.h"
static_assert(CHASSIS_MPC_KNOTS==8, "Regenerate for changed horizon");
static_assert(CHASSIS_MPC_RHO==1.0f, "Regenerate for changed rho");
static_assert(CHASSIS_MPC_TS_S==0.004f, "Regenerate for changed sample time");
static_assert(CHASSIS_MPC_LEG_LENGTH_M==0.24f, "Regenerate for changed legs");
static TinySolution solution;
static TinyCache cache;
static TinySettings settings;
static TinyWorkspace work;
static TinySolver solver={&solution,&settings,&cache,&work};
template<typename T> static void load(T& m,const tinytype* data,int rows,int cols) {
    m.resize(rows,cols);
    for(int r=0;r<rows;++r) for(int c=0;c<cols;++c) m(r,c)=data[r*cols+c];
}
static const tinytype data_A[] = {
    (tinytype)1, (tinytype)0.004000000000000001, (tinytype)0, (tinytype)0, (tinytype)-0.00014581201265395695, (tinytype)-1.9439776531344506e-07, (tinytype)-0.00014581201265395695, (tinytype)-1.9439776531344506e-07, (tinytype)0, (tinytype)0,
    (tinytype)0, (tinytype)1, (tinytype)0, (tinytype)0, (tinytype)-0.072923118081059043, (tinytype)-0.00014581201265395698, (tinytype)-0.072923118081059043, (tinytype)-0.00014581201265395698, (tinytype)0, (tinytype)0,
    (tinytype)0, (tinytype)0, (tinytype)1, (tinytype)0.004000000000000001, (tinytype)-5.4905672067757013e-05, (tinytype)-7.3200949712447348e-08, (tinytype)5.4905672067757013e-05, (tinytype)7.3200949712447348e-08, (tinytype)0, (tinytype)0,
    (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)1, (tinytype)-0.027459036083203447, (tinytype)-5.4905672067757007e-05, (tinytype)0.027459036083203447, (tinytype)5.4905672067757007e-05, (tinytype)0, (tinytype)0,
    (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)1.0013820424146178, (tinytype)0.0040018425534309975, (tinytype)2.6613080942142157e-05, (tinytype)3.547757240990626e-08, (tinytype)0, (tinytype)0,
    (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)0.69118039240573459, (tinytype)1.0013820424146178, (tinytype)0.013312667998264973, (tinytype)2.6613080942142157e-05, (tinytype)0, (tinytype)0,
    (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)2.6613080942142143e-05, (tinytype)3.5477572409906233e-08, (tinytype)1.0013820424146178, (tinytype)0.0040018425534309975, (tinytype)0, (tinytype)0,
    (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)0.013312667998264966, (tinytype)2.6613080942142143e-05, (tinytype)0.69118039240573426, (tinytype)1.0013820424146178, (tinytype)0, (tinytype)0,
    (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)-6.8964860369388688e-05, (tinytype)-9.194308677356876e-08, (tinytype)-6.8964860369388674e-05, (tinytype)-9.194308677356872e-08, (tinytype)1.0002330238331389, (tinytype)0.0040003106936177778,
    (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)0, (tinytype)-0.034491862314095519, (tinytype)-6.8964860369388688e-05, (tinytype)-0.034491862314095512, (tinytype)-6.8964860369388661e-05, (tinytype)0.11651644136754728, (tinytype)1.0002330238331389
};
static const tinytype data_B[] = {
    (tinytype)4.476204023397862e-05, (tinytype)4.476204023397862e-05, (tinytype)-9.2281696605994533e-06, (tinytype)-9.2281696605994533e-06,
    (tinytype)0.022385385878876264, (tinytype)0.022385385878876264, (tinytype)-0.0046151678012221605, (tinytype)-0.0046151678012221605,
    (tinytype)-6.6383781440209759e-05, (tinytype)6.6383781440209759e-05, (tinytype)-3.4748773297091451e-06, (tinytype)3.4748773297091451e-06,
    (tinytype)-0.033190600236731739, (tinytype)0.033190600236731739, (tinytype)-0.001737831054384305, (tinytype)0.001737831054384305,
    (tinytype)-0.00032075687647587588, (tinytype)-3.8636698572605735e-05, (tinytype)8.7466880458643702e-05, (tinytype)1.6842921351630421e-06,
    (tinytype)-0.16041545530066889, (tinytype)-0.019323508820427229, (tinytype)0.043743514756475942, (tinytype)0.00084253386732117215,
    (tinytype)-3.8636698572605714e-05, (tinytype)-0.00032075687647587582, (tinytype)1.6842921351630418e-06, (tinytype)8.7466880458643675e-05,
    (tinytype)-0.019323508820427219, (tinytype)-0.16041545530066886, (tinytype)0.00084253386732117182, (tinytype)0.043743514756475921,
    (tinytype)-4.8541027828266423e-06, (tinytype)-4.8541027828266414e-06, (tinytype)-0.00010325601295762022, (tinytype)-0.00010325601295762022,
    (tinytype)-0.0024250808643992328, (tinytype)-0.0024250808643992323, (tinytype)-0.05163052366805513, (tinytype)-0.05163052366805513
};
static const tinytype data_Q[] = {
    (tinytype)21,
    (tinytype)2,
    (tinytype)801,
    (tinytype)6,
    (tinytype)501,
    (tinytype)2,
    (tinytype)501,
    (tinytype)2,
    (tinytype)2501,
    (tinytype)19
};
static const tinytype data_R[] = {
    (tinytype)5,
    (tinytype)5,
    (tinytype)2.5,
    (tinytype)2.5
};
static const tinytype data_P[] = {
    (tinytype)8421.8433849357662, (tinytype)6488.1877320370531, (tinytype)2.4357404981856234e-11, (tinytype)-8.3040460514590473e-13, (tinytype)2955.778224451612, (tinytype)846.39336347942697, (tinytype)2955.7782244515647, (tinytype)846.39336347942765, (tinytype)655.15172890321162, (tinytype)138.69282709840189,
    (tinytype)6488.1877320370531, (tinytype)9552.4045955873135, (tinytype)7.1790642414686127e-11, (tinytype)-1.9108154097491058e-12, (tinytype)3889.1870600157508, (tinytype)1251.1843315376168, (tinytype)3889.1870600157781, (tinytype)1251.1843315376193, (tinytype)933.71630223488614, (tinytype)207.43964124718394,
    (tinytype)2.4357404981856234e-11, (tinytype)7.1790642414686127e-11, (tinytype)37474.047565583198, (tinytype)2911.8042893636284, (tinytype)-8049.9737175751525, (tinytype)-559.28208548586394, (tinytype)8049.9737175750233, (tinytype)559.28208548587804, (tinytype)1.2797853289744703e-10, (tinytype)1.0729195309977513e-12,
    (tinytype)-8.3040460514590473e-13, (tinytype)-1.9108154097491058e-12, (tinytype)2911.8042893636284, (tinytype)542.45070298571852, (tinytype)-1676.042284941554, (tinytype)-117.62027520027542, (tinytype)1676.0422849415427, (tinytype)117.62027520027449, (tinytype)1.7236691630758685e-11, (tinytype)-1.321099876698001e-13,
    (tinytype)2955.778224451612, (tinytype)3889.1870600157508, (tinytype)-8049.9737175751525, (tinytype)-1676.042284941554, (tinytype)21692.430904038552, (tinytype)1345.8185240795242, (tinytype)-4595.758360453623, (tinytype)67.801603229978383, (tinytype)1186.2519853024933, (tinytype)174.42343797096379,
    (tinytype)846.39336347942697, (tinytype)1251.1843315376168, (tinytype)-559.28208548586394, (tinytype)-117.62027520027542, (tinytype)1345.8185240795242, (tinytype)224.18464875385209, (tinytype)67.801603229979364, (tinytype)132.7097246253328, (tinytype)187.05239831685267, (tinytype)34.835945340077259,
    (tinytype)2955.7782244515647, (tinytype)3889.1870600157781, (tinytype)8049.9737175750233, (tinytype)1676.0422849415427, (tinytype)-4595.758360453623, (tinytype)67.801603229979364, (tinytype)21692.430904038687, (tinytype)1345.8185240795317, (tinytype)1186.2519853024874, (tinytype)174.42343797096208,
    (tinytype)846.39336347942765, (tinytype)1251.1843315376193, (tinytype)559.28208548587804, (tinytype)117.62027520027449, (tinytype)67.801603229978383, (tinytype)132.7097246253328, (tinytype)1345.8185240795317, (tinytype)224.18464875385271, (tinytype)187.05239831685424, (tinytype)34.835945340077217,
    (tinytype)655.15172890321162, (tinytype)933.71630223488614, (tinytype)1.2797853289744703e-10, (tinytype)1.7236691630758685e-11, (tinytype)1186.2519853024933, (tinytype)187.05239831685267, (tinytype)1186.2519853024874, (tinytype)187.05239831685424, (tinytype)68136.093529909675, (tinytype)1205.2986373663418,
    (tinytype)138.69282709840189, (tinytype)207.43964124718394, (tinytype)1.0729195309977513e-12, (tinytype)-1.321099876698001e-13, (tinytype)174.42343797096379, (tinytype)34.835945340077259, (tinytype)174.42343797096208, (tinytype)34.835945340077217, (tinytype)1205.2986373663418, (tinytype)134.52128895096155
};
static const tinytype data_K[] = {
    (tinytype)-1.3072385376354188, (tinytype)-2.0976569463765959, (tinytype)-3.5141797012233993, (tinytype)-0.29787920480469871, (tinytype)-13.794929695078707, (tinytype)-1.2036845828469813, (tinytype)-1.2241526855884899, (tinytype)-0.29766132344086987, (tinytype)-2.9852692680528183, (tinytype)-0.36711592969375617,
    (tinytype)-1.307238537635437, (tinytype)-2.0976569463766777, (tinytype)3.5141797012235205, (tinytype)0.29787920480471353, (tinytype)-1.2241526855886184, (tinytype)-0.29766132344088342, (tinytype)-13.794929695078869, (tinytype)-1.2036845828469929, (tinytype)-2.9852692680526598, (tinytype)-0.36711592969375639,
    (tinytype)0.2056555705990194, (tinytype)0.32319547636459944, (tinytype)-11.294396822557628, (tinytype)-2.3297569845133279, (tinytype)13.264718103695937, (tinytype)0.94787323569565329, (tinytype)-10.089481487023924, (tinytype)-0.70566751276892536, (tinytype)-20.608808698715979, (tinytype)-2.1033648339621558,
    (tinytype)0.20565557059901945, (tinytype)0.32319547636462193, (tinytype)11.294396822557571, (tinytype)2.329756984513323, (tinytype)-10.089481487023933, (tinytype)-0.7056675127689237, (tinytype)13.264718103696005, (tinytype)0.94787323569565696, (tinytype)-20.608808698715912, (tinytype)-2.1033648339621558
};
static const tinytype data_inv[] = {
    (tinytype)0.16839993792306127, (tinytype)-0.0028814369736901344, (tinytype)0.0091499817194925902, (tinytype)-0.0055620294682765005,
    (tinytype)-0.0028814369736901006, (tinytype)0.16839993792306113, (tinytype)-0.0055620294682764354, (tinytype)0.0091499817194926284,
    (tinytype)0.0091499817194925937, (tinytype)-0.0055620294682764285, (tinytype)0.34051394139910657, (tinytype)-0.028704422142104783,
    (tinytype)-0.0055620294682765152, (tinytype)0.0091499817194926267, (tinytype)-0.02870442214210478, (tinytype)0.34051394139910651
};
static const tinytype data_AmBKt[] = {
    (tinytype)1.0001208249770284, (tinytype)0.060424348136555055, (tinytype)1.2126325513435113e-18, (tinytype)6.0718403373102198e-16, (tinytype)-0.00048814756675129398, (tinytype)-0.24413106988424965, (tinytype)-0.00048814756675129897, (tinytype)-0.24413106988425218, (tinytype)2.9779408078449542e-05, (tinytype)0.014895891284910286,
    (tinytype)0.0041937558146404545, (tinytype)1.0968969230843297, (tinytype)5.3529879192099016e-18, (tinytype)2.6755283488574007e-15, (tinytype)-0.00078269768487848699, (tinytype)-0.39144069584733593, (tinytype)-0.00078269768487851203, (tinytype)-0.39144069584734836, (tinytype)4.6379267749052026e-05, (tinytype)0.023199527942238552,
    (tinytype)-5.9492868081635076e-18, (tinytype)-2.9833668505063213e-15, (tinytype)0.99945493763860183, (tinytype)-0.27253097432425055, (tinytype)-2.2558409501712165e-05, (tinytype)-0.011281752087885889, (tinytype)2.2558409501761019e-05, (tinytype)0.011281752087910419, (tinytype)-5.2702486864859855e-18, (tinytype)-2.6148401361085417e-15,
    (tinytype)-7.0689084598902295e-19, (tinytype)-3.5490754473038828e-16, (tinytype)0.0039442600644868887, (tinytype)0.97212897271527099, (tinytype)0.00011581485004307487, (tinytype)0.057920503047099926, (tinytype)-0.00011581485004306903, (tinytype)-0.057920503047096991, (tinytype)-4.3198209333393136e-19, (tinytype)-2.2125874428969892e-16,
    (tinytype)0.00055577437935811034, (tinytype)0.27793908635133685, (tinytype)-0.00080824840688806558, (tinytype)-0.40410501717124486, (tinytype)0.995766696757723, (tinytype)-2.1171391229415439, (tinytype)-3.8879040537850696e-05, (tinytype)-0.019453379988006635, (tinytype)0.00018599324328570614, (tinytype)0.093024777692471722,
    (tinytype)6.9244023600999047e-05, (tinytype)0.034580215490994277, (tinytype)-5.4472599721283467e-05, (tinytype)-0.027252787017824645, (tinytype)0.0035225328307895809, (tinytype)0.76167181271546835, (tinytype)-8.1821796639220589e-05, (tinytype)-0.040912511342910539, (tinytype)1.7629566836461287e-05, (tinytype)0.0087953582215546996,
    (tinytype)0.00055577437935811261, (tinytype)0.27793908635133796, (tinytype)0.00080824840688808466, (tinytype)0.40410501717125441, (tinytype)-3.8879040537816707e-05, (tinytype)-0.01945337998798953, (tinytype)0.995766696757723, (tinytype)-2.1171391229415701, (tinytype)0.00018599324328571381, (tinytype)0.09302477769247558,
    (tinytype)6.9244023600998979e-05, (tinytype)0.034580215490994243, (tinytype)5.4472599721285121e-05, (tinytype)0.027252787017825467, (tinytype)-8.1821796639216577e-05, (tinytype)-0.040912511342908534, (tinytype)0.0035225328307895779, (tinytype)0.76167181271546669, (tinytype)1.7629566836461497e-05, (tinytype)0.0087953582215548141,
    (tinytype)-0.00011310968017750394, (tinytype)-0.056573411620299892, (tinytype)-1.0766865625757657e-17, (tinytype)-5.3728491862766529e-15, (tinytype)0.00076441286652411376, (tinytype)0.38229614085713015, (tinytype)0.00076441286652415225, (tinytype)0.38229614085714936, (tinytype)0.99594807538934438, (tinytype)-2.0260497679654685,
    (tinytype)-5.9546990609099729e-06, (tinytype)-0.0029786998148959871, (tinytype)1.5986575449877313e-20, (tinytype)7.218421747104945e-18, (tinytype)5.5578334910417753e-05, (tinytype)0.027795689845112732, (tinytype)5.5578334910417679e-05, (tinytype)0.027795689845112669, (tinytype)0.0035623765236054866, (tinytype)0.78125679651603019
};

void whu_tiny_init() {
    load(work.Adyn,data_A,10,10);
    load(work.Bdyn,data_B,10,4);
    load(work.Q,data_Q,10,1);
    load(work.R,data_R,4,1);
    load(cache.Pinf,data_P,10,10);
    load(cache.Kinf,data_K,4,10);
    load(cache.Quu_inv,data_inv,4,4);
    load(cache.AmBKt,data_AmBKt,10,10);
    work.nx=10; work.nu=4; work.N=CHASSIS_MPC_KNOTS;
    cache.rho=CHASSIS_MPC_RHO;
    cache.APf=tinyVector::Zero(10); cache.BPf=tinyVector::Zero(4);
    work.fdyn=tinyVector::Zero(10);
    settings.abs_pri_tol=CHASSIS_MPC_TOL; settings.abs_dua_tol=CHASSIS_MPC_TOL;
    settings.max_iter=CHASSIS_MPC_MAX_ITER; settings.check_termination=1;
    settings.en_input_bound=1; settings.adaptive_rho=0;
    work.Xref=tinyMatrix::Zero(10,work.N);
    work.Uref=tinyMatrix::Zero(4,work.N-1);
    work.u_min=tinyMatrix::Zero(4,work.N-1);
    work.u_max=tinyMatrix::Zero(4,work.N-1);
    for(int i=0;i<4;++i) {
        tinytype limit=i<2?CHASSIS_MPC_WHEEL_LIMIT_NM:CHASSIS_MPC_HIP_LIMIT_NM;
        work.u_min.row(i).setConstant(-limit); work.u_max.row(i).setConstant(limit);
    }
    work.x=tinyMatrix::Zero(10,8);
    work.u=tinyMatrix::Zero(4,7);
    work.q=tinyMatrix::Zero(10,8);
    work.r=tinyMatrix::Zero(4,7);
    work.p=tinyMatrix::Zero(10,8);
    work.d=tinyMatrix::Zero(4,7);
    work.v=tinyMatrix::Zero(10,8);
    work.vnew=tinyMatrix::Zero(10,8);
    work.z=tinyMatrix::Zero(4,7);
    work.znew=tinyMatrix::Zero(4,7);
    work.g=tinyMatrix::Zero(10,8);
    work.y=tinyMatrix::Zero(4,7);
    solution.x=tinyMatrix::Zero(10,work.N); solution.u=tinyMatrix::Zero(4,work.N-1);
}
void whu_tiny_reset() {
    work.x.setZero();
    work.u.setZero();
    work.q.setZero();
    work.r.setZero();
    work.p.setZero();
    work.d.setZero();
    work.v.setZero();
    work.vnew.setZero();
    work.z.setZero();
    work.znew.setZero();
    work.g.setZero();
    work.y.setZero();
    solution.solved=0; solution.iter=0; solution.x.setZero(); solution.u.setZero();
}
TinySolver *whu_tiny_solver() { return &solver; }
uint32_t whu_tiny_payload_bytes() {
    uint32_t elements=0;
    elements+=(uint32_t)work.Adyn.size();
    elements+=(uint32_t)work.Bdyn.size();
    elements+=(uint32_t)work.Q.size();
    elements+=(uint32_t)work.R.size();
    elements+=(uint32_t)cache.Pinf.size();
    elements+=(uint32_t)cache.Kinf.size();
    elements+=(uint32_t)cache.Quu_inv.size();
    elements+=(uint32_t)cache.AmBKt.size();
    elements+=(uint32_t)work.fdyn.size();
    elements+=(uint32_t)cache.APf.size();
    elements+=(uint32_t)cache.BPf.size();
    elements+=(uint32_t)work.Xref.size();
    elements+=(uint32_t)work.Uref.size();
    elements+=(uint32_t)work.u_min.size();
    elements+=(uint32_t)work.u_max.size();
    elements+=(uint32_t)work.x.size();
    elements+=(uint32_t)work.u.size();
    elements+=(uint32_t)work.q.size();
    elements+=(uint32_t)work.r.size();
    elements+=(uint32_t)work.p.size();
    elements+=(uint32_t)work.d.size();
    elements+=(uint32_t)work.v.size();
    elements+=(uint32_t)work.vnew.size();
    elements+=(uint32_t)work.z.size();
    elements+=(uint32_t)work.znew.size();
    elements+=(uint32_t)work.g.size();
    elements+=(uint32_t)work.y.size();
    elements+=(uint32_t)solution.x.size();
    elements+=(uint32_t)solution.u.size();
    return elements*sizeof(tinytype);
}
