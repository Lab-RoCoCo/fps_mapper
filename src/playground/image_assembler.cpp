#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
using namespace std;


enum Action {Declare, Clear, Resize, Load, Show, Compose, Create, Bgr2Rgb, PutText, ScaleColor, Save};

struct Rule{
  Action action;
  std::vector<string> args;
  cv::Mat*  getSymbol(std::string& name, std::map<std::string, cv::Mat>& symbols) {
    std::map<std::string, cv::Mat>::iterator it = symbols.find(name);
      if (it==symbols.end()) {
	cerr <<"Unknown Symbol: " << name << " you must declare it before using" << endl;
	throw std::runtime_error("Declare requires at least one arg");
	return 0;
      }
      return &it->second;
  }


  void execute( std::map<std::string, cv::Mat>& symbols, 
		std::vector<std::string>& work_args, 
		int run_number, 
		int rule_number) {
    switch (action){

    case Declare: {
      if (args.size()<1)
	throw std::runtime_error("Declare requires at least one arg (name)");
      symbols.insert(std::make_pair(args[0], cv::Mat()));
      cerr << "declared symbol " << args[0] << endl;
      return;
    }

    case Create: {
      if (args.size()<3)
	throw std::runtime_error("create requres at least three args (rows cols)");
      int fr = atoi(args[1].c_str());
      int fc = atoi(args[2].c_str());
      cv::Mat m;
      m.create(fr, fc, CV_8UC3);
      symbols.insert(make_pair(args[0], m));
      cerr << "created symbol " << args[0] << " with size" << m.rows << " " << m.cols << endl;
      return;
    }

    case Load: {
      if (args.size()<2)
	throw std::runtime_error("Load requires at least two args (name, id)");
      cv::Mat& dest = *getSymbol(args[0], symbols);
      std::string filename; 
      if (args[1][0]=='$') {
	string v=args[1].substr(1);
	int idx = atoi(v.c_str());
	if (idx < work_args.size()){
	  filename = work_args[idx];
	}
      } else
	filename = args[1];
      cerr << "loading symbol " << args[0] << " from file [" << filename << "]" << endl;
      dest = cv::imread(filename.c_str(), CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);
      return;
    }

    case Clear: {
      if (args.size()<1)
	throw std::runtime_error("Clear requires at least one arg (name)");
      cv::Mat& dest = *getSymbol(args[0], symbols);
      cerr << "clearing symbol " << args[0] << endl;
      return;
    }

    case PutText: {
      if (args.size()<8)
	throw std::runtime_error("Clear requires at least 9 args (name r c scale text color_r color_g  color_b thickness)");
      cv::Mat& dest = *getSymbol(args[0], symbols);
      int fr = atoi(args[1].c_str());
      int fc = atoi(args[2].c_str());
      double scale = atof(args[3].c_str());
      int r = atoi(args[4].c_str());
      int g = atoi(args[5].c_str());
      int b = atoi(args[6].c_str());
      double thickness = atof(args[7].c_str());
      std::string message = args[8];
      for (size_t i = 0; i<message.length(); i++){
	if (message[i]=='_')
	  message[i]=' ';
      }
      cerr << "putting text " << args[0] << " new size " << fr << " " << fc  <<  " " << scale << " " 
	   << message << endl;
      cv::putText(dest, message, cv::Point(fc, fr), cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(r,g,b), thickness);
      return;
    }

    case Resize: {
      if (args.size()<3)
	throw std::runtime_error("Resize requires at least three args (name, rows, cols)");
      cv::Mat& m = *getSymbol(args[0], symbols);
      int fr = atoi(args[1].c_str());
      int fc = atoi(args[2].c_str());
      cerr << "resizing " << args[0] << " new size " << fr << " " << fc  << endl;
      cv::Mat m2;
      cv::resize(m, m2,  cv::Size(fc, fr), 0,0, cv::INTER_LINEAR);
      m=m2;
      return;
    }

    case Compose: {
      if (args.size()<4)
	throw std::runtime_error("Compose requires at least four args (dest, src, origin_row, origin_col)");
      cv::Mat& dest = *getSymbol(args[0], symbols);
      cv::Mat& src = *getSymbol(args[1], symbols);
      int origin_row = atoi(args[2].c_str());
      int origin_col = atoi(args[3].c_str());
      cerr << "composing " << args[0] << " " << args[1] << " " << origin_row << " " << origin_col << endl;

      cv::Mat rrange = dest.rowRange(origin_row, origin_row+src.rows);
      cv::Mat crange = rrange.colRange(origin_col, origin_col+src.cols);
      cv::Mat tmp, tmp2;
      if (src.type()==CV_16UC1) {
	convertScaleAbs(src, tmp2, 1./255);
	cv::cvtColor(tmp2, tmp, CV_GRAY2RGB);
      } else {
	src.convertTo(tmp, dest.type());
      }
      tmp.copyTo(crange); 
      return;
    }
    case Show: {
      if (args.size()<1){
	throw std::runtime_error("Show requires at least one arf (name)");
      }
      cv::Mat& dest = *getSymbol(args[0], symbols);
      cv::imshow(args[0], dest);
      return;
    }
    case Bgr2Rgb: {
      if (args.size()<1){
	throw std::runtime_error("Show requires at least one arg (name)");
      }
      cv::Mat& dest = *getSymbol(args[0], symbols);
      cv::cvtColor(dest, dest, CV_BGR2RGB);
      return;
    }
    case ScaleColor: {
      if (args.size()<1){
	throw std::runtime_error("Scale requires at least two args (name scale)");
      }
      cv::Mat& dest = *getSymbol(args[0], symbols);
      double scale = atof(args[1].c_str());
      dest*= scale;
      return;
    }

    case Save: {
      if (args.size()<2){
	throw std::runtime_error("save requires two args");
      }
      cv::Mat& src = *getSymbol(args[0], symbols);
      std::string filename; 
      if (args[1][0]=='$') {
	string v=args[1].substr(1);
	int idx = atoi(v.c_str());
	if (idx < work_args.size()){
	  filename = work_args[idx];
	}
      } else
	filename = args[1];
      
      char run_str[1024];
      char rule_str[1024];
      sprintf(run_str,"%05d", run_number);
      sprintf(rule_str,"%05d", rule_number);
      
      size_t found = filename.find("$RULENUM");
      if (found!=std::string::npos){
	filename=filename.replace(found, strlen("$RULENUM"), std::string(rule_str));
      }
      
      found = filename.find("$RUN");
      if (found!=std::string::npos){
	filename=filename.replace(found, strlen("$RUN"), run_str);
      }
      cerr << "saving " << args[0] << " in file [" << filename << "]" << endl;
      cv::imwrite(filename, src);
      return;
    }

    default:;
    }
  }
};
    

  std::map<std::string, Action> action_tag_map;
  
  void initRuleTags() {
    action_tag_map.insert(make_pair(std::string("declare"), Declare));
    action_tag_map.insert(make_pair(std::string("clear"), Clear));
    action_tag_map.insert(make_pair(std::string("resize"), Resize));
    action_tag_map.insert(make_pair(std::string("compose"), Compose));
    action_tag_map.insert(make_pair(std::string("load"), Load));
    action_tag_map.insert(make_pair(std::string("show"), Show));
    action_tag_map.insert(make_pair(std::string("create"), Create));
    action_tag_map.insert(make_pair(std::string("bgr2rgb"), Bgr2Rgb));
    action_tag_map.insert(make_pair(std::string("putText"), PutText));
    action_tag_map.insert(make_pair(std::string("scaleColor"), ScaleColor));
    action_tag_map.insert(make_pair(std::string("save"), Save));
  }
  
  Rule* parseRule (istream& is){
    std::string tag;
    is >> tag;
    std::map<std::string, Action>::iterator it = action_tag_map.find(tag);
    if (it == action_tag_map.end()) {
      cerr << "fail" << endl;
      return 0;
    } 
    Rule* r = new Rule;
    r->action = it->second;
    while (is) {
      std::string s;
      is >> s;
      if (is)
	r->args.push_back(s);
    }
    return r;
  }


  void parseRules(std::vector<Rule*>& rules, istream& is) {
    while (is) {
      char buf[32000];
      is.getline(buf, 32000);
      istringstream ls(buf);
      Rule* r = parseRule(ls);
      if (r)
	rules.push_back(r);
    }
  }


void runRules(std::vector<Rule*>& rules, std::vector<std::string>& parsed_line, int run_num) {
  cerr << "*****************************" << endl;
  cerr << "Run num: " << run_num << endl;
  std::map<std::string, cv::Mat> symbols;    
  for(size_t i = 0; i<rules.size(); i++) {
    Rule* r = rules[i];
    cerr << "Rule: " << i << endl;
    r->execute(symbols, parsed_line, run_num, i);
  }
  cerr << "*****************************" << endl;
  cerr << endl;
}


  int main (int argc, char** argv) {
    cerr << "usage : " << argv[0] << "work.rules input_list.txt" << endl;
    if (argc<3)
      return 0;
    ifstream is(argv[1]);
    if (! is)
      return 0;
    initRuleTags();
    std::vector<Rule*> rules;
    parseRules(rules, is);
    for (size_t i = 0; i<rules.size(); i++ ) {
      Rule* r = rules[i];
      cerr << r->action << " " ;
      for (size_t j=0; j<r->args.size(); j++)
	cerr << r->args[j] <<  " ";
      cerr << endl;
    }
    ifstream is2(argv[2]);
    int i = 0;
    while (is2) {
      char buf[32000];
      is2.getline(buf, 32000);
      std::vector<std::string> filenames;
      filenames.clear();
      istringstream ls(buf);
      while (ls) {
	std::string fname;
	ls >> fname;
	filenames.push_back(fname);
      }
      runRules(rules, filenames, i);
      i++;
      cv::waitKey(1);
    }
  }  /*
 
bool addToCanvas(cv::Mat& canvas, cv::Mat& src, int r, int c, int fr, int fc) {
  if (c+fc>canvas.cols) {
    cerr << "c+fc" << c+fc << endl;
    throw std::runtime_error ("canvad cannot contain image, cols overflow");
  }

  if (r+fr>canvas.rows) {
    cerr << "r+fr" << r+fr << endl;
    throw std::runtime_error ("canvad cannot contain image, rows overflow");
  }
  cv::Mat src1, src2;
  src.convertTo(src1, CV_8UC3);

  cv::resize(src1, src2,  cv::Size(fc, fr), 0,0, cv::INTER_LINEAR);
  for (int rr=0; rr<src2.rows; rr++) {
    const cv::Vec3b* srcptr = src2.ptr<cv::Vec3b>(rr);
    cv::Vec3b* destptr = canvas.ptr<cv::Vec3b>(rr+r) + c;
    cerr << "rr+r" << rr+r <<endl;
    cerr << "src2.cols" << src2.cols <<endl;

    for(int i =0; i< src2.cols; i++) {
      *destptr = *srcptr;
      destptr++;
      srcptr++;
    }
  }

}

  void processShit(cv::Mat& canvas, const std::vector<std::string>& filenames){
    std::string 
      outer_name=filenames[0], 
      cloud_name=filenames[1], 
      depth_name=filenames[2], 
      rgb_name=filenames[3];
    cv::Mat outer_image=cv::imread(outer_name.c_str(), CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);
    cv::Mat cloud_image=cv::imread(cloud_name.c_str(), CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);
    cv::Mat depth_image=cv::imread(depth_name.c_str(), CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);
    cv::Mat rgb_image=cv::imread(rgb_name.c_str(), CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);

    cerr << "i1" << endl;
    addToCanvas(canvas, cloud_image,0,0,480,640);
    cerr << "i2" << endl;
  
    addToCanvas(canvas, outer_image,0,640,240,320);
    cerr << "i3" << endl;
    addToCanvas(canvas, rgb_image,240,640,240,320);
    cerr << "i4" << endl;
    cv::imshow("canvas", canvas);
  } 

int main(int argc, char** argv) {
  ifstream is(argv[1]);
  while (is) {
    char buf[32000];
    is.getline(buf, 32000);
    std::vector<std::string> filenames;
    filenames.clear();
    istringstream ls(buf);
    while (ls) {
      std::string fname;
      ls >> fname;
      filenames.push_back(fname);
    }
    cv::Mat canvas;
    canvas.create(480, 960, CV_8UC3);
    processShit(canvas, filenames);
    cv::waitKey(1);
  }
}
*/
