#ifndef __glob_h__
#define __glob_h__

#include <map>

namespace glob {

	inline void assert(bool exp) 
    {
        if (!exp) {
            __asm int 3;
        }
    }

	class ExecTimeStorer {
	public:
		bool				   init(const char* logFileName);
        static ExecTimeStorer* inst() { return m_inst; }

		void addRecord(const char* funcName, float execTime);
		void flush	  (void);
	private:
        static ExecTimeStorer* m_inst;

        ExecTimeStorer(const char* logFileName);

		struct _FuncDescr {
			int		nCalls;
			float	totalExecTime;
		};

		std::map<const char*, _FuncDescr> m_profileMap;
	};
	
	class ExecTimeCalculator {
	public:
		explicit ExecTimeCalculator(const char* funcName);
		~ExecTimeCalculator();
	private:
        
	};
    
#define PROFILE_FUNC(func_name) ExecTimeCalculator _##func_name_exec_tile_calculator(##func_name);
    
    

} // namespace glob

#endif //__glob_h__