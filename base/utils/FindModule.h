#ifndef FIND_MODULE_H
#define FIND_MODULE_H

#include <omnetpp.h>
#include <vector>

using namespace std;

/**
 * @brief Provides method templates to find omnet modules.
 *
 * @ingroup baseUtils
 * @ingroup utils
 */
template<typename T = cModule*>
class FindModule
{
	public:
	/**
	 * @brief Returns a pointer to a vector containing pointers to sub modules of the passed module with
	 * the type of this template.
	 *
	 * Returns NULL if no matching submodule could be found.
	 */
	static vector<T>* findAllSubModules(cModule *top)
	{
		vector<T>* answer = new vector<T>();
		for (cModule::SubmoduleIterator i(top); !i.end(); i++)
		{
			cModule *sub = i();
			T ret = dynamic_cast<T>(sub);
			if (ret!=NULL)
				answer->push_back(ret);
				//return ret;
			vector<T>* retSubModule = findAllSubModules(sub);
			if (retSubModule!=NULL) {
				typename vector<T>::iterator it;
				for(it = retSubModule->begin(); it != retSubModule->end(); it++) {
					answer->push_back(*it);
				}
			}
		}
		return answer;
	}
		/**
		 * @brief Returns a pointer to a sub module of the passed module with
		 * the type of this template.
		 *
		 * Returns NULL if no matching submodule could be found.
		 */
		static T findSubModule(cModule *top)
		{
			T ret;
			for (cModule::SubmoduleIterator i(top); !i.end(); i++)
			{
				cModule *sub = i();
				ret = dynamic_cast<T>(sub);
				if (ret!=NULL)
					return ret;
				ret = findSubModule(sub);
				if (ret!=NULL)
					return ret;
			}
			return NULL;
		}

		/**
		 * @brief Returns a pointer to the module with the type of this
		 * template.
		 *
		 * Returns NULL if no module of this type could be found.
		 */
		static T findGlobalModule() {return findSubModule(simulation.getSystemModule());}

		/**
		 * @brief Returns a pointer to the host module of the passed module.
		 *
		 * Assumes that every host module is a direct sub module of the
		 * simulation.
		 */
		static cModule* findHost(cModule *m) {
			 cModule *parent = m->getParentModule();
			cModule *node = m;

			// all nodes should be a sub module of the simulation which has no parent module!!!
			while( parent->getParentModule() != NULL ){
			node = parent;
			parent = node->getParentModule();
			}

			return node;
		}
};

#endif
