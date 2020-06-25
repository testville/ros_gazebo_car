namespace movment_model {
    template <typename State, typename Data>
    class MovmentModel
    {
        public:
            virtual State* GetDerivatives(State* state, Data* data);
            virtual State* SolveEq(State* state, Data* data);
    };
}