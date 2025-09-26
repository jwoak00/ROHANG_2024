// #include "log.h"

// int create_file(const char *path)
// {
//    FILE *fp = fopen(path, "w");
//    fclose(fp);
// }

// void save_timestamp(const char *path, int wpt, unsigned long long time)
// {
//    FILE *fp = fopen(path, "a+");
//    fprintf(fp, "%d,%lld\n", wpt, time);
//    fclose(fp);
// }

// void save_output(const char *path, int wpt, double data, double altitude, unsigned long long time)
// {
//    FILE *fp = fopen(path, "a+");
//    fprintf(fp, "%lld,<%d>,Remain(m),%f,Height(m),%f\n", time, wpt, data, altitude);
//    fclose(fp);
// }

// void save_log(const char *path, int autopilot, double *llh, int *datetime, int mission_index)
// {
//     int i = 0;

//     FILE *fp = fopen(path, "a+");
//     fprintf(fp, "%d,", autopilot);
//     for(i=0; i<3; i++){
//         fprintf(fp, "%f,", llh[i]);
//     }
//     for(i=0; i<7; i++){
//         fprintf(fp, "%d,", datetime[i]);
//     }
//     fprintf(fp, "%d\n", mission_index);
//     fclose(fp);
// }
