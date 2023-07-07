#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace models
	{
		class Model;
	}
	namespace results
	{
		class Path;
		class What;
		class Canvas;
		class Bounding_Box;
	}
	namespace analysis
	{
		class Assembler;
	}
}

namespace fea
{
	namespace results
	{
		class Results
		{
			friend class models::Model;
			friend class analysis::Assembler;

		public:
			//constructors
			Results(void);

			//destructor
			virtual ~Results(void);

			//serialization
			virtual void load(FILE*);
			virtual void save(FILE*) const;

			//data
			bool status(void) const;
			unsigned steps(void) const;

			Path* path(void) const;
			What* what(void) const;
			Canvas* canvas(void) const;
			Bounding_Box* bounding_box(void) const;
			static const models::Model* model(void);

			//data
			bool read(void);
			void clear(void);

		private:
			//data
			bool m_status;
			unsigned m_steps;

			Path* m_path;
			What* m_what;
			Canvas* m_canvas;
			Bounding_Box* m_bounding_box;
			static const models::Model* m_model;
		};
	}
}