
#include <string>
#include <vector>

#include <database_interface/db_class.h>

// all database classes must inherit from database_interface::DBClass
class Student : public database_interface::DBClass
{

//fields are made public in this toy example for easy access, but you
//can treat them as you would any member data of your C++ classes
public:

  //key requirement: all fields that are to be stored in the 
  //database must be wrapped as database_interface::DBField<>, 
  //templated on the type of data they hold
  
  database_interface::DBField<int> student_id_;

  database_interface::DBField<std::string> student_first_name_;

  database_interface::DBField<std::string> student_last_name_;

  database_interface::DBField< std::vector<std::string> > student_majors_;

  database_interface::DBField<double> student_gpa_;
  
  //key requirement: all fields must be initialized in the constructor
  //a field constructor takes the following arguments:
  // - the type of serialization used (TEXT for all fields in this toy example)
  // - the owner of the field ( usually "this", or the instance of the DBClass 
  //   that owns that field)
  // - the name of the table column corresponding to that field
  // - the name of the table in which the field is stored
  // - whether it is allowed to modify en entry in the database using a reference 
  //   to this field
  Student() : 
    student_id_(database_interface::DBFieldBase::TEXT, 
		this, "student_id", "student", true),
    student_first_name_(database_interface::DBFieldBase::TEXT, 
			this, "student_first_name", "student", true),
    student_last_name_(database_interface::DBFieldBase::TEXT, 
		       this, "student_last_name", "student", true),
    student_majors_(database_interface::DBFieldBase::TEXT, 
		    this, "student_majors", "student", true),
    student_gpa_(database_interface::DBFieldBase::TEXT, 
		 this, "student_gpa", "student", true)
  {
    //finally, all fields must be registered with the DBClass itself

    //one field MUST be a primary key
    //all instances of DBClass have a primary_key_field_ pointer, 
    //which must be set on construction
    primary_key_field_ = &student_id_;

    //all other fields go into the fields_ array of the DBClass
    fields_.push_back(&student_first_name_);
    fields_.push_back(&student_last_name_);
    fields_.push_back(&student_majors_);
    fields_.push_back(&student_gpa_);

    //optional: let all fields be read automatically when an instance 
    //of a student is retrieved from the database
    setAllFieldsReadFromDatabase(true);
    //optional: let all fields be written automatically when an instance 
    //of a student is saved the database
    setAllFieldsWriteToDatabase(true);
    //(these options are usful if you have a very large field (e.g. a 
    // binary bitmap with the picture of the student) which you do not 
    //want retrieved automatically whenever you get a student info 
    //from the database
  }
};

class Grade : public database_interface::DBClass
{
public:
  database_interface::DBField<int> grade_id_;
  database_interface::DBField<int> student_id_;
  database_interface::DBField<std::string> grade_subject_;
  database_interface::DBField<double> grade_grade_;

  Grade() :
    grade_id_(database_interface::DBFieldBase::TEXT, 
		this, "grade_id", "grade", true),
    student_id_(database_interface::DBFieldBase::TEXT, 
		this, "student_id", "grade", true),
    grade_subject_(database_interface::DBFieldBase::TEXT, 
		   this, "grade_subject", "grade", true),
    grade_grade_(database_interface::DBFieldBase::TEXT, 
		 this, "grade_grade", "grade", true)  
  {
    primary_key_field_ = &grade_id_;
    fields_.push_back(&student_id_);
    fields_.push_back(&grade_subject_);
    fields_.push_back(&grade_grade_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);
  }
};

class StudentWithPhoto : public database_interface::DBClass
{
public:  
  database_interface::DBField<int> student_id_;
  database_interface::DBField<std::string> student_first_name_;
  database_interface::DBField<std::string> student_last_name_;
  database_interface::DBField< std::vector<std::string> > student_majors_;
  database_interface::DBField<double> student_gpa_;
  database_interface::DBField< std::vector<char> > student_photo_;
  
  StudentWithPhoto() : 
    student_id_(database_interface::DBFieldBase::TEXT, 
		this, "student_id", "student", true),
    student_first_name_(database_interface::DBFieldBase::TEXT, 
			this, "student_first_name", "student", true),
    student_last_name_(database_interface::DBFieldBase::TEXT, 
		       this, "student_last_name", "student", true),
    student_majors_(database_interface::DBFieldBase::TEXT, 
		    this, "student_majors", "student", true),
    student_gpa_(database_interface::DBFieldBase::TEXT, 
		 this, "student_gpa", "student", true),
    student_photo_(database_interface::DBFieldBase::BINARY, 
		 this, "student_photo", "student", true)
  {
    primary_key_field_ = &student_id_;

    fields_.push_back(&student_first_name_);
    fields_.push_back(&student_last_name_);
    fields_.push_back(&student_majors_);
    fields_.push_back(&student_gpa_);
    fields_.push_back(&student_photo_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);

    student_photo_.setReadFromDatabase(false);
    student_photo_.setWriteToDatabase(false);
  }
};


class GradeWithSequence : public database_interface::DBClass
{
public:
  database_interface::DBField<int> grade_id_;
  database_interface::DBField<int> student_id_;
  database_interface::DBField<std::string> grade_subject_;
  database_interface::DBField<double> grade_grade_;

  GradeWithSequence() :
    grade_id_(database_interface::DBFieldBase::TEXT, 
		this, "grade_id", "grade", true),
    student_id_(database_interface::DBFieldBase::TEXT, 
		this, "student_id", "grade", true),
    grade_subject_(database_interface::DBFieldBase::TEXT, 
		   this, "grade_subject", "grade", true),
    grade_grade_(database_interface::DBFieldBase::TEXT, 
		 this, "grade_grade", "grade", true)  
  {
    primary_key_field_ = &grade_id_;
    fields_.push_back(&student_id_);
    fields_.push_back(&grade_subject_);
    fields_.push_back(&grade_grade_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);

    grade_id_.setSequenceName("grade_id_seq");
    grade_id_.setWriteToDatabase(false);
  }
};


#include <boost/shared_ptr.hpp>
#include <database_interface/postgresql_database.h>

int main(int argc, char **argv)
{
  database_interface::PostgresqlDatabase 
    database("wgs36.willowgarage.com", "5432",
	     "willow", "willow", "students");
  if (!database.isConnected())
  {
    std::cerr << "Database failed to connect \n";
    return -1;
  }
  std::cerr << "Database connected successfully \n";

  std::vector< boost::shared_ptr<Student> > students;
  if (!database.getList(students))
  {
    std::cerr << "Failed to get list of students\n";
    return -1;
  }
  std::cerr << "Retrieved " << students.size() << " student(s) \n";

  std::cerr << "Students: \n";
  for (size_t i=0; i<students.size(); i++)
  {
    std::cerr << students[i]->student_last_name_.data() 
	      << ", " 
	      << students[i]->student_first_name_.data()
	      << ": \n";
    std::cerr << "  GPA: " << students[i]->student_gpa_.data() << "\n";
    std::cerr << "  Major(s): ";
    for (size_t j=0; j<students[i]->student_majors_.data().size(); j++)
    {
      if (j!=0) std::cerr << ", ";
      std::cerr << students[i]->student_majors_.data().at(j);
    }
    std::cerr << "\n";
  }

  std::vector< boost::shared_ptr<Grade> > grades;
  std::string where_clause("student_id=1");
  database.getList(grades, where_clause);
  std::cerr << "Student with id 1 has " << grades.size() << " grade(s) on record\n";

  grades[0]->grade_grade_.data() = 2.5;
  if (!database.saveToDatabase( &(grades[0]->grade_grade_) ) )
    std::cerr << "Failed to modify grade\n";
  else
    std::cerr << "Grade modified successfully\n";

  //we have forgotten a grade
  grades[0]->grade_grade_.data() = 0.0;
  //reload it from the database
  if (!database.loadFromDatabase( &(grades[0]->grade_grade_) ) )
    std::cerr << "Failed to load grade field from database\n";
  else
    std::cerr << "Grade field (re)loaded, its value is "
              << grades[0]->grade_grade_.data() 
              << "\n";

  Grade new_grade;
  new_grade.student_id_.data() = 1;
  new_grade.grade_subject_.data() = "astrology";
  new_grade.grade_grade_.data() = 4.0;
  new_grade.grade_id_.data() = 5;
  if (!database.insertIntoDatabase(&new_grade)) 
    std::cerr << "Grade insertion failed\n";
  else 
    std::cerr << "Grade insertion succeeded\n";

  //get the students, but photos will not be loaded by default
  std::vector< boost::shared_ptr<StudentWithPhoto> > students_with_photos;
  database.getList(students_with_photos);

  //now get the photo for the first student retrieved
  database.loadFromDatabase( &(students_with_photos[0]->student_photo_) );
  std::cerr << "The photo has " 
	    << students_with_photos[0]->student_photo_.data().size() 
	    << " bytes \n";

  //insert a grade with automatic key generation
  GradeWithSequence new_grade_seq;
  new_grade_seq.student_id_.data() = 2;
  new_grade_seq.grade_subject_.data() = "mythology";
  new_grade_seq.grade_grade_.data() = 4.0;
  database.insertIntoDatabase(&new_grade_seq);
  std::cerr << "The newly inserted grade was assigned grade_id=" 
	    << new_grade_seq.grade_id_.data() 
	    << "\n";

  return 0;
}
