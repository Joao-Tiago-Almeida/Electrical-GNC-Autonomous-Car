classdef MinHeap < Heap
%--------------------------------------------------------------------------
% Class:        MinHeap < Heap (& handle)
%               
% Constructor:  H = MinHeap(n);
%               H = MinHeap(n,x0);
%               
% Properties:   (none)
%               
% Methods:                   H.InsertKey(key);
%               sx         = H.Sort();
%               min        = H.ExtractMin();
%                          = H.UpdateElement(key);
%               count      = H.Count();
%               capacity   = H.Capacity();
%               bool       = H.IsEmpty();
%               bool       = H.IsFull();
%                            H.Clear();
%                            H.Print();
%               
% Description:  This class implements a min-heap of numeric keys
%--------------------------------------------------------------------------
    %
    % Public methods
    %
    methods (Access = public)
        %
        % Constructor
        %
        function this = MinHeap(varargin)
            %----------------------- Constructor --------------------------
            % Syntax:       H = MinHeap(n);
            %               H = MinHeap(n,x0);
            %               
            % Inputs:       n is the maximum number of keys that H can hold
            %               
            %               x0 is a vector (of length <= n) of numeric keys
            %               to insert into the heap during initialization
            %               
            % Description:  Creates a min-heap with capacity n
            %--------------------------------------------------------------
            
            % Call base class constructor
            this = this@Heap(varargin{:});
            
            % Construct the min heap
            this.BuildMinHeap();
        end
        
        %
        % Insert key
        %
        function InsertKey(this,key)
            %------------------------ InsertKey ---------------------------
            % Syntax:       H.InsertKey(key);
            %               
            % Inputs:       key is a number
            %               
            % Description:  Inserts key into H
            %--------------------------------------------------------------
            
            this.SetLength(this.k + 1);
            this.x(this.k) = structfun(@(x) [], this.x(this.k), 'UniformOutput', false);
            this.DecreaseKey(this.k,key);
        end
        
        %
        % Sort the heap
        %
        function sx = Sort(this)
            %-------------------------- Sort ------------------------------
            % Syntax:       sx = H.Sort();
            %               
            % Outputs:      sx is a vector taht contains the sorted
            %               (ascending order) keys in H
            %               
            % Description:  Returns the sorted values in H
            %--------------------------------------------------------------
            
            % Sort the heap
            nk = this.k; % virtual heap size during sorting procedure
            for i = this.k:-1:2
                this.Swap(1,i);
                nk = nk - 1;
                this.MinHeapify(1,nk);
            end
            this.x(1:this.k) = flipud(this.x(1:this.k));
            sx = this.x(1:this.k);
        end
        
        %
        % Extract minimum element
        %
        function min_node = ExtractMin(this)
            %------------------------ ExtractMin --------------------------
            % Syntax:       min = H.ExtractMin();
            %               
            % Outputs:      min is the minimum key in H
            %               
            % Description:  Returns the minimum key in H and extracts it
            %               from the heap
            %--------------------------------------------------------------
            
            if (this.IsEmpty() == true)
                min_node = [];
            else
            	this.SetLength(this.k - 1);
                min_node = this.x(1);
                this.x(1) = this.x(this.k + 1);
                this.MinHeapify(1);
            end

        end
        
        %
        % Update Element
        %
        function UpdateElement(this,key)
            %----------------------- UpdateElement ------------------------
            % Syntax:       H.UpdateElement();
            %               
            % Inputs:       key is a struct
            %
            % Description:  Updates the position of a node
            %--------------------------------------------------------------
          
            if (this.IsEmpty() == true)
                InsertKey(this,key);
                has_update = true;
            else
                % search in the heap for the node
                for i = 1:this.k
                    if(key.index ~= this.x(i).index); continue; end
                    
                    if(key.previous ~= this.x(i).previous)
                        if(key.cost < this.x(i).cost)
                            DecreaseKey(this,i,key)
                        else
%                             this.x(i)=key;
%                             this.MinHeapify(i);
                            return;
                        end
                    % found node in the matrix
                    elseif(key.cost < this.x(i).cost)
                        DecreaseKey(this,i,key)
                    end
                    return
                end
                disp("Ops, something is wrong, I should have update the key ... but I did not find it")
            end
        end

        %
        % Prints the heap
        %
        function Print(this)
            %-------------------------- Print -----------------------------
            % Syntax:       H.Print();
            %               
            % Outputs:      
            %               
            % Description:  Prints the heap in the Command Window
            %--------------------------------------------------------------
            size_level = 1;
            count=0;
            for i=1:this.k
                fprintf(sprintf('%d. ',log2(size_level)))

                for j=1:size_level

                    % break neighbours in lines
                    count=count+1;
                    if(count>2)
                        fprintf('\n   ');
                        count=1;
                    end

                    idx = size_level+j-1;
                    if(idx>this.k);fprintf('\n');return;end
                    S = sprintf('(%d - %.5f)\t',this.x(idx).index,this.x(idx).cost); 
                    fprintf(S);
                end
                fprintf('\n\n');
                size_level = size_level*2;
                count=0;
            end
        end
    end
    %
    % Private methods
    %
    methods (Access = private)
        %
        % Decrease key at index i
        %
        function DecreaseKey(this,i,key)
            if (i > this.k)
                % Index overflow error
                MinHeap.IndexOverflowError();
            elseif (key.cost > this.x(i).cost)
                % Decrease key error
                MinHeap.DecreaseKeyError();
            end
            this.x(i) = key;
            while ((i > 1) && (this.x(Heap.parent(i)).cost > this.x(i).cost))
                this.Swap(i,Heap.parent(i));
                i = Heap.parent(i);
            end
        end
        %
        % Increase key at index i
        %
        function IncreaseKey(this,i,key)
            if (i > this.k)
                % Index overflow error
                MinHeap.IndexOverflowError();
            elseif (key.cost < this.x(i).cost)
                % Increase key error
                MinHeap.IncreaseKeyError();
            end
            this.x(i) = key;
            while ((i > 1) && (this.x(Heap.parent(i)).cost < this.x(i).cost))
                this.Swap(i,Heap.parent(i));
                i = Heap.parent(i);
            end
        end
        %
        % Build the min heap
        %
        function BuildMinHeap(this)
            for i = floor(this.k / 2):-1:1
                this.MinHeapify(i);
            end
        end
        
        %
        % Maintain the min heap property at a given node
        %
        function MinHeapify(this,i,size)
            % Parse inputs
            if (nargin < 3)
                size = this.k;
            end
            
            ll = Heap.left(i);
            rr = Heap.right(i);
            if ((ll <= size) && (this.x(ll).cost < this.x(i).cost))
                smallest = ll;
            else
                smallest = i;
            end
            if ((rr <= size) && (this.x(rr).cost < this.x(smallest).cost))
                smallest = rr;
            end
            if (smallest ~= i)
                this.Swap(i,smallest);
                this.MinHeapify(smallest,size);
            end
        end
    end
    
    %
    % Private static methods
    %
    methods (Access = private, Static = true)
        %
        % Decrease key error
        %
        function DecreaseKeyError()
            error('You can only decrease keys in MinHeap');
        end
        
        %
        % Index overflow error
        %
        function IndexOverflowError()
            error('MinHeap index overflow');
        end
    end
end