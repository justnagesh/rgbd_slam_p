//
// Created by nagesh
//

#include "fileOps.h"

fileOps::fileOps()
{
    fstream File;

    // used during stream parsing only to skip unuseful data
    float time_stamp;
    float q1, q2, q3, q4,   /* quaternion fractions */
    tr1, tr2, tr3;          /* translation part */

    string  depth_name;
    string  rgb_name;

    File.open ( "/home/nagesh/Desktop/rgbd_dataset_freiburg1_room/associate.txt", std::ios::in );
    if ( ! File.is_open() )
    {
        cerr << "Can't read association file: "  << endl;
    }
    else
    {
        while( ! File.eof())
        {

            string cur_line ("");
            getline ( File, cur_line );
            stringstream magic_stream ( cur_line );
            cout <<cur_line <<endl;

            // skip all unuseful data - according to format of particular file
            magic_stream >> time_stamp;
            magic_stream >> tr1 >> tr2 >> tr3;
            magic_stream >> q1 >> q2 >> q3 >> q4;
            magic_stream >> time_stamp;

            magic_stream >> depth_name;
            if( ! cur_line.empty() )
            {
                depth_names.push_back ( depth_name );
                magic_stream >> time_stamp;
                magic_stream >> rgb_name;
                rgb_names.push_back ( rgb_name );

                char tmp_str[128];
                sprintf ( tmp_str, "%05d.pcd", pcd_file_names.size() );

                pcd_file_names.push_back(tmp_str);
            }
        }
    }

    cout << "Total parsed "<< pcd_file_names.size()<< " files"<< endl;


}